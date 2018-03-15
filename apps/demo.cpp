#include <iostream>
#include <string>
#include <fstream>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/viz/vizcore.hpp>
#include <kfusion/kinfu.hpp>
#include <io/capture.hpp>
#include "rply.h"


using namespace kfusion;

struct KinFuApp
{
    static void KeyboardCallback(const cv::viz::KeyboardEvent& event, void* pthis)
    {
        KinFuApp& kinfu = *static_cast<KinFuApp*>(pthis);

        if(event.action != cv::viz::KeyboardEvent::KEY_DOWN)
            return;

        if(event.code == 't' || event.code == 'T')
            kinfu.take_cloud(*kinfu.kinfu_);

        if(event.code == 'i' || event.code == 'I')
            kinfu.iteractive_mode_ = !kinfu.iteractive_mode_;
    }

    //KinFuApp(OpenNISource& source) : exit_ (false),  iteractive_mode_(false), capture_ (source), pause_(false)
    // Constructor changed for reading ply data from disk and merge them
    KinFuApp() : exit_ (false),  iteractive_mode_(false), pause_(false)
    {
        KinFuParams params = KinFuParams::default_params();
        kinfu_ = KinFu::Ptr( new KinFu(params) );

        //capture_.setRegistration(true);

        cv::viz::WCube cube(cv::Vec3d::all(0), cv::Vec3d(params.volume_size), true, cv::viz::Color::apricot());
        viz.showWidget("cube", cube, params.volume_pose);
        viz.showWidget("coor", cv::viz::WCoordinateSystem(0.1));
        viz.registerKeyboardCallback(KeyboardCallback, this);
    }

    void show_depth(const cv::Mat& depth)
    {
        cv::Mat display;
        //cv::normalize(depth, display, 0, 255, cv::NORM_MINMAX, CV_8U);
        depth.convertTo(display, CV_8U, 255.0/4000);
        cv::imshow("Depth", display);
    }

    void show_raycasted(KinFu& kinfu)
    {
        const int mode = 3;
        if (iteractive_mode_)
            kinfu.renderImage(view_device_, viz.getViewerPose(), mode);
        else
            kinfu.renderImage(view_device_, mode);

        view_host_.create(view_device_.rows(), view_device_.cols(), CV_8UC4);
        view_device_.download(view_host_.ptr<void>(), view_host_.step);
        cv::imshow("Scene", view_host_);
    }

    void take_cloud(KinFu& kinfu)
    {
        cuda::DeviceArray<Point> cloud = kinfu.tsdf().fetchCloud(cloud_buffer);

        cv::Mat cloud_host(1, (int)cloud.size(), CV_32FC4);
        cloud.download(cloud_host.ptr<Point>());
        viz.showWidget("cloud", cv::viz::WCloud(cloud_host));
        std::cout << cloud_host.col(cloud_host.cols-1).at<float>(0) << std::endl;

        std::ofstream output_file("output_cloud.ply",std::ofstream::out);

        output_file << "ply" << "\n";
        output_file << "format ascii 1.0" << "\n";
        output_file << "element vertex " << cloud_host.cols-1 << "\n";
        output_file << "property float x" << "\n";
        output_file << "property float y" << "\n";
        output_file << "property float z" << "\n";
        output_file << "property float red" << "\n";
        output_file << "property float green" << "\n";
        output_file << "property float blue" << "\n";
        output_file << "end_header" << "\n";

        for(int i = 0;i<cloud_host.cols;i++)
            output_file << cloud_host.col(i).at<float>(0) << " " << cloud_host.col(i).at<float>(1) << " " << cloud_host.col(i).at<float>(2) << " 200 200 200" << "\n";
        //std::cout << RobotRQuaternion.coeffs() << "\n";
        //result rotation = RobotRMat (as Eigen::Matrix3D), rotation = RobotRQuaternion (as Eigen::Quaterniond) , translation = RobotTranslationVec (as Eigen::Vector3D), translation = TrackerTranslationVector (as double[3])

        output_file.close();

        //viz.showWidget("cloud", cv::viz::WPaintedCloud(cloud_host));
    }

    bool execute()
    {
        KinFu& kinfu = *kinfu_;

        cv::Mat depth, image;
        double time_ms = 0;
        bool has_image = false;

        for (int i = 1; !exit_ && !viz.wasStopped(); ++i)
        {


            //obtaining dept image
            //bool has_frame = capture_.grab(depth, image);

            //obraining cloud data
            std::string path = "/home/mirec/catkin_ws/clouds/cloud_from_phoxi/cloud" + std::to_string(i)+ ".ply";
            std::cout << path << std::endl;
            p_ply ply = ply_open(path.c_str(), NULL, 0, NULL);
            long int size;

            bool has_frame = ply && ply_read_header(ply);

            if (!has_frame)
                return std::cout << "Can't grab" << std::endl, false;

            if(!ply_read(ply,&size))
                return std::cout << "Can't read" << std::endl, false;


            printf("Velkost: %ld \n",size);
            cv::Mat cloud(1,size,CV_32FC4);
            //depth_device_.upload(depth.data, depth.step, depth.rows, depth.cols);

            // reading cloud data
            /*ply_set_read_cb(ply, "vertex", "x", this->vertex_X, NULL, 0);
            ply_set_read_cb(ply, "vertex", "y", this->vertex_Y, NULL, 0);
            ply_set_read_cb(ply, "vertex", "z", this->vertex_Z, NULL, 1);*/

            cloud_device_.upload(cloud.data,cloud.step,cloud.rows,cloud.cols);
            {
                SampledScopeTime fps(time_ms); (void)fps;
                //has_image = kinfu(depth_device_);
                has_image = kinfu(cloud_device_);
            }

            if (has_image)
                show_raycasted(kinfu);

            //show_depth(depth);
            //cv::imshow("Image", image);

            if (!iteractive_mode_)
                viz.setViewerPose(kinfu.getCameraPose());

            int key = cv::waitKey(pause_ ? 0 : 3);

            switch(key)
            {
                case 't': case 'T' : take_cloud(kinfu); break;
                case 'i': case 'I' : iteractive_mode_ = !iteractive_mode_; break;
                case 27: exit_ = true; break;
                case 32: pause_ = !pause_; break;
            }

            //exit_ = exit_ || i > 100;
            viz.spinOnce(3, true);
        }
        return true;
    }

    static int vertex_X(p_ply_argument argument) {
        long eol;

        ply_get_argument_user_data(argument, NULL, &eol);

        //printf("%g", ply_get_argument_value(argument));
        //if (eol) printf("\n");
        //else printf(" ");
        //this->cloud.col(2).at<float>(0);

        return 1;
    }

    static int vertex_Y(p_ply_argument argument) {
        long eol;

        ply_get_argument_user_data(argument, NULL, &eol);
        //printf("%g", ply_get_argument_value(argument));
        //if (eol) printf("\n");
        //else printf(" ");
        return 1;
    }
    static int vertex_Z(p_ply_argument argument) {
        long eol;

        ply_get_argument_user_data(argument, NULL, &eol);
        //printf("%g", ply_get_argument_value(argument));
        //if (eol) printf("\n");
        //else printf(" ");
        return 1;
    }

    static int face_cb(p_ply_argument argument) {
        long length, value_index;
        ply_get_argument_property(argument, NULL, &length, &value_index);
        switch (value_index) {
            case 0:
            case 1:
                printf("%g ", ply_get_argument_value(argument));
                break;
            case 2:
                printf("%g\n", ply_get_argument_value(argument));
                break;
            default:
                break;
        }
        return 1;
    }

    bool pause_ /*= false*/;
    bool exit_, iteractive_mode_;
    //OpenNISource& capture_;
    KinFu::Ptr kinfu_;
    cv::viz::Viz3d viz;

    cv::Mat view_host_;
    cuda::Image view_device_;
    cuda::Depth depth_device_;
    cuda::Cloud cloud_device_;
    cuda::DeviceArray<Point> cloud_buffer;
};


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


int main (int argc, char* argv[])
{
    int device = 0;
    cuda::setDevice (device);
    cuda::printShortCudaDeviceInfo (device);
    cuda::printCudaDeviceInfo (device);

    if(cuda::checkIfPreFermiGPU(device))
        return std::cout << std::endl << "Kinfu is not supported for pre-Fermi GPU architectures, and not built for them by default. Exiting..." << std::endl, 1;

    OpenNISource capture;
    //capture.open (0);
    //capture.open("/home/mirec/OpenNI/Platform/Linux/Bin/x64-Release/mr.oni");
    //capture.open("d:/onis/reg20111229-180846.oni");
    //capture.open("d:/onis/white1.oni");
    //capture.open("/media/Main/onis/20111013-224932.oni");
    //capture.open("20111013-225218.oni");
    //capture.open("d:/onis/20111013-224551.oni");
    //capture.open("d:/onis/20111013-224719.oni");

    KinFuApp app;

    // executing
    try { app.execute (); }
    catch (const std::bad_alloc& /*e*/) { std::cout << "Bad alloc" << std::endl; }
    catch (const std::exception& /*e*/) { std::cout << "Exception" << std::endl; }

    return 0;
}
