#include <iostream>
#include <string>
#include <fstream>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/viz/vizcore.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/viz/vizcore.hpp>
#include <kfusion/kinfu.hpp>
#include <io/capture.hpp>
#include <PhoXi.h>
/*
#include "/home/mirec/PhotoneoMain_REPO/Common/PhoLibraryExport.h"
#include "/home/mirec/PhotoneoMain_REPO/Common/FileSystemTool/PathGenerator.h"
#include "/home/mirec/PhotoneoMain_REPO/Common/Duration.h"
#include "/home/mirec/PhotoneoMain_REPO/Common/CameraCalibration/PhotoneoCameraCalibration.h"*/
#include <PhoXiInterface.h>
#include <PhoXiDataTypes.h>
#include "rply.h"


//#include "CameraCalibration/PhotoneoCameraCalibrationTool.h"


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

    //------------------------------------------------------------------------------------------------------------

#define TexturePath "User/Texture.Mat"
#define DepthPath "User/DepthMap.Mat"
#define PointCloudPath "User/PointCloud.Mat"

//------------------------------------------------------------------------------------------------------------


/*---------------------------------------------------------------------------------------*/
/*DATA CLASSES*/
/*---------------------------------------------------------------------------------------*/


    // otazky na tomasa
    // ?? UINT64 SPECIALNY TYP ???

    class RGBDFrame
    {
    public:
        uint64 m_colorSizeBytes;					//compressed byte size
        uint64 m_depthSizeBytes;					//compressed byte size
        unsigned char* m_colorCompressed;			//compressed color data
        unsigned char* m_depthCompressed;			//compressed depth data
        uint64 m_timeStampColor;					//time stamp color (convection: in microseconds)
        uint64 m_timeStampDepth;					//time stamp depth (convention: in microseconds)
        //mat4f m_cameraToWorld;						//camera trajectory: from current frame to base frame
        cv::Matx44f m_cameraToWorld;						//camera trajectory: from current frame to base frame

        cv::Mat tex;
        cv::Mat depth;

        void saveToFile(std::ofstream& out) const {
            out.write((const char*)&m_cameraToWorld, sizeof(cv::Matx44f));
            out.write((const char*)&m_timeStampColor, sizeof(uint64));
            out.write((const char*)&m_timeStampDepth, sizeof(uint64));
            out.write((const char*)&m_colorSizeBytes, sizeof(uint64));
            out.write((const char*)&m_depthSizeBytes, sizeof(uint64));
            out.write((const char*)m_colorCompressed, m_colorSizeBytes);
            out.write((const char*)m_depthCompressed, m_depthSizeBytes);
        }
    };

    class SensorData
    {
    public:
        unsigned int	m_versionNumber = 4;
        std::string		m_sensorName;

        std::vector<RGBDFrame> m_frames;

        unsigned int m_colorWidth;
        unsigned int m_colorHeight;
        unsigned int m_depthWidth;
        unsigned int m_depthHeight;
        float m_depthShift;	//conversion from float[m] to ushort (typically 1000.0f)

        enum COMPRESSION_TYPE_COLOR {
            TYPE_RAW = 0,
            TYPE_PNG = 1,
            TYPE_JPEG = 2
        };
        enum COMPRESSION_TYPE_DEPTH {
            TYPE_RAW_USHORT = 0,
            TYPE_ZLIB_USHORT = 1,
            TYPE_OCCI_USHORT = 2
        };

        COMPRESSION_TYPE_COLOR m_colorCompressionType = TYPE_RAW;
        COMPRESSION_TYPE_DEPTH m_depthCompressionType = TYPE_RAW_USHORT;

        cv::Matx44f m_intrinsic;
        cv::Matx44f m_extrinsic;

        //! saves a .sens file
        void saveToFile(const std::string& filename) const {
            std::ofstream out(filename, std::ios::binary);

            out.write((const char*)&m_versionNumber, sizeof(unsigned int));
            uint64 strLen = m_sensorName.size();
            out.write((const char*)&strLen, sizeof(uint64));
            out.write((const char*)&m_sensorName[0], strLen*sizeof(char));

            //m_calibrationColor.saveToFile(out);
            out.write((const char*)&m_intrinsic, sizeof(cv::Matx44f));
            out.write((const char*)&m_extrinsic, sizeof(cv::Matx44f));
            //m_calibrationDepth.saveToFile(out);
            out.write((const char*)&m_intrinsic, sizeof(cv::Matx44f));
            out.write((const char*)&m_extrinsic, sizeof(cv::Matx44f));

            out.write((const char*)&m_colorCompressionType, sizeof(COMPRESSION_TYPE_COLOR));
            out.write((const char*)&m_depthCompressionType, sizeof(COMPRESSION_TYPE_DEPTH));
            out.write((const char*)&m_colorWidth, sizeof(unsigned int));
            out.write((const char*)&m_colorHeight, sizeof(unsigned int));
            out.write((const char*)&m_depthWidth, sizeof(unsigned int));
            out.write((const char*)&m_depthHeight, sizeof(unsigned int));
            out.write((const char*)&m_depthShift, sizeof(float));

            uint64 numFrames = m_frames.size();
            out.write((const char*)&numFrames, sizeof(uint64));
            for (size_t i = 0; i < m_frames.size(); i++) {
                m_frames[i].saveToFile(out);
            }

            /*
            UINT64 numIMUFrames = m_frames.size();
            out.write((const char*)&numIMUFrames, sizeof(UINT64));
            for (size_t i = 0; i < m_IMUFrames.size(); i++) {
            m_IMUFrames[i].saveToFile(out);
            }*/
        }
    };


/*---------------------------------------------------------------------------------------*/
    void Project3DPointsToImagePoints(cv::InputArray ObjectPoints,
                                                                   cv::OutputArray ImagePoints,
                                                                   bool WorldCoordinate,cv::Mat CameraMatrix) const {
        std::vector<float> DistortionCoefficients;
        cv::projectPoints(ObjectPoints,
                              cv::Mat::zeros(3, 1, cv::DataType<float>::type),
                              cv::Mat::zeros(3, 1, cv::DataType<float>::type),
                              CameraMatrix,
                              DistortionCoefficients,
                              ImagePoints);

    }
    template<class T>
    bool ReprojectToDepthMap(cv::Mat& cameraMatrix,
            /*const photoneoTools::CameraCalibration<T>& cameraCalibration*/ cv::Mat& InputCloud, const cv::Size& OutputResolution, cv::Mat& OutputDepthMap, const cv::Mat& InputTexture, cv::Mat& OutputTexture, T IntegrationThreshold) {
        std::vector<cv::Point_<T>> ProjectedCoordinates;
        cv::Mat InputCloudRaw = InputCloud;
        cv::Mat InputCloudWorking = InputCloudRaw;
        if (InputCloudRaw.cols > 1 && InputCloudRaw.rows > 1) {
            InputCloudWorking = InputCloudRaw.reshape(3, InputCloudRaw.size().area());
        }

        Project3DPointsToImagePoints(InputCloudWorking, ProjectedCoordinates, false,cameraMatrix);

        cv::Size_<T> OutputResolutionInternal;
        OutputResolutionInternal.width = OutputResolution.width;
        OutputResolutionInternal.height = OutputResolution.height;
        OutputResolutionInternal.width -= (T)1.0;
        OutputResolutionInternal.height -= (T)1.0;

        cv::Mat& OutputDepthMapRef = OutputDepthMap;
        if (OutputDepthMapRef.size() != OutputResolution || OutputDepthMapRef.type() != cv::DataType<T>::type) {
            OutputDepthMapRef = cv::Mat::zeros(OutputResolution, cv::DataType<T>::type);
        }
        else {
            OutputDepthMapRef.setTo((T)0.0);
        }
        cv::Mat& OutputTextureRef = OutputTexture;
        if (OutputTextureRef.size() != OutputResolution || OutputTextureRef.type() != cv::DataType<T>::type) {
            OutputTextureRef = cv::Mat::zeros(OutputResolution, cv::DataType<T>::type);
        }
        else {
            OutputTextureRef.setTo((T)0.0);
        }

        cv::Mat OutputWeights = cv::Mat::zeros(OutputResolution, cv::DataType<T>::type);

        cv::Mat InputTextureRaw = InputTexture;
        cv::Mat InputTextureWorking = InputTextureRaw;
        if (InputTextureRaw.cols > 1 && InputTextureRaw.rows > 1) {
            InputTextureWorking = InputTextureRaw.reshape(1, InputTextureRaw.size().area());
        }

        const cv::Point relativeNeighbour[4] = { cv::Point(0, 0), cv::Point(1, 0), cv::Point(0, 1), cv::Point(1, 1) };



        for (std::size_t PointID = 0; PointID < ProjectedCoordinates.size(); ++PointID) {
            const cv::Point3_<T>& point3D = InputCloudWorking.at<cv::Point3_<T>>(PointID);
            const cv::Point_<T>& imagePoint = ProjectedCoordinates[PointID];
            const T& texture = InputTextureWorking.at<T>(PointID);
            if (imagePoint.x >= (T)0.0 && imagePoint.y >= (T)0.0 && imagePoint.x < OutputResolutionInternal.width && imagePoint.y < OutputResolutionInternal.height) {
                cv::Point intPoint(floor(imagePoint.x), floor(imagePoint.y));
                T weightX[2];
                weightX[0] = imagePoint.x - floor(imagePoint.x);
                weightX[1] = (T)1.0 - weightX[0];
                T weightY[2];
                weightY[0] = imagePoint.y - floor(imagePoint.y);
                weightY[1] = (T)1.0 - weightY[0];
                for (int neighbourID = 0; neighbourID < 4; ++neighbourID) {
                    cv::Point pointOfInterest = intPoint + relativeNeighbour[neighbourID];
                    T& currentDepth = OutputDepthMapRef.at<T>(pointOfInterest);
                    T& currentTexture = OutputTextureRef.at<T>(pointOfInterest);
                    T& currentWeight = OutputWeights.at<T>(pointOfInterest);


                    if (currentWeight >(T)0.0) {
                        T depth = currentDepth / currentWeight;
                        T integrationThreshold = depth * IntegrationThreshold;
                        if (point3D.z < depth - integrationThreshold) continue;
                        if (point3D.z > depth + integrationThreshold) {
                            currentDepth = (T)0.0;
                            currentTexture = (T)0.0;
                            currentWeight = (T)0.0;
                        }
                    }

                    T weight = weightX[relativeNeighbour[neighbourID].x] * weightY[relativeNeighbour[neighbourID].y];

                    currentDepth += weight * point3D.z;
                    currentTexture += weight * texture;
                    currentWeight += weight;

                }
            }
        }

        for (int y = 0; y < OutputWeights.rows; ++y) {
            T* currentDepth = OutputDepthMapRef.ptr<T>(y);
            T* currentTexture = OutputTextureRef.ptr<T>(y);
            T* currentWeight = OutputWeights.ptr<T>(y);
            for (int x = 0; x < OutputWeights.cols; ++x) {
                if (*currentWeight >(T)0.0) {
                    *currentDepth /= *currentWeight;
                    *currentTexture /= *currentWeight;
                }
                ++currentDepth;
                ++currentTexture;
                ++currentWeight;
            }
        }
        return true;
    }

    bool Scan(cv::Mat& depthxxx/*pho::api::PPhoXi PhoXiDevice, int index, SensorData& sd*/)
    {
        std::cout << "Triggering the " << "1" << "-th frame" << std::endl;
        std::string CustomMessage = "Hello World - Software Trigger Frame Number: 1";
        //int FrameID = PhoXiDevice->TriggerFrame(true, false, CustomMessage);
        //int FrameID = PhoXiDevice->TriggerFrame(true,false);
        if (true) {
            /*std::cout << "Frame retrieved" << std::endl;
            std::cout << "    Sensor Position: " << Frame->Info.SensorPosition.x << "; " << Frame->Info.SensorPosition.y << "; " << Frame->Info.SensorPosition.z << std::endl;
            //std::cout << "    Frame Custom Message: " << Frame->CustomMessage << std::endl;


            //pho::api::PhoXiRawAccessHandler RawAccessHandler(PhoXiDevice);


            //pho::PDataManager WholeDM = RawAccessHandler.GetLastOutput();
            //if (WholeDM) {
            if(true){
                std::cout << "DM ok" << std::endl;

                /*double FrameTimestamp = -1;
                std::string FrameStartTimePath = "User/FrameStartTime.double";
                if (WholeDM->isExistingLeaf(FrameStartTimePath) && !WholeDM->operator[](FrameStartTimePath).isNull()) {
                    FrameTimestamp = WholeDM->GetLeaf(FrameStartTimePath).Ref<double>();

                    if ((index > 0) && (lastTimestamp > FrameTimestamp)) return false;
                    lastTimestamp = FrameTimestamp;
                }
                else {
                    std::cout << "FrameTimestamp missing!!!" << std::endl;
                    return false;
                }*/


                pho::api::PFrame frame = scanner->GetSpecificFrame(scanner->TriggerFrame());
                if (!(((bool)frame) && (frame->Successful))){
                    printf ("no depth\n");
                    return false;
                }

                //cv::Mat Texture = WholeDM->GetLeaf(TexturePath).Ref<cv::Mat>();
                cv::Mat Texture;
                cv::Mat(frame->Texture.Size.Height, frame->Texture.Size.Width, CV_32FC1, (void*)frame->Texture.GetDataPtr()).copyTo(Texture);
                //sd.m_colorWidth = Texture.cols;
                //sd.m_colorHeight = Texture.rows;
                std::cout << "Texture rows:" << Texture.rows << std::endl;
                cv::Mat Depth; //= WholeDM->GetLeaf(DepthPath).Ref<cv::Mat>();
                cv::Mat(frame->DepthMap.Size.Height, frame->DepthMap.Size.Width, CV_32FC1, (void*)frame->DepthMap.GetDataPtr()).copyTo(Depth);

                cv::Mat PointCloud;//= WholeDM->GetLeaf(PointCloudPath).Ref<cv::Mat>();
                cv::Mat(frame->PointCloud.Size.Height, frame->PointCloud.Size.Width, CV_32FC3, (void*)frame->PointCloud.GetDataPtr()).copyTo(PointCloud);

                //photoneoTools::CameraCalibration32 cameraCalibration;
                //cameraCalibration.SetCameraMatrix(1824.885927 / 3.0*2.0, 1824.728925 / 3.0*2.0, 878.237084 / 3.0*2.0, 594.768363 / 3.0*2.0);
                cv::Mat CameraMatrix = cv::Mat::eye(3,3,CV_32FC1);
                CameraMatrix.at<float>(0,0) = 1824.885927 / 3.0*2.0; //fx
                CameraMatrix.at<float>(1,1) = 1824.728925 / 3.0*2.0;//fy
                CameraMatrix.at<float>(0,2) = 878.237084 / 3.0*2.0; //cx
                CameraMatrix.at<float>(1,2) = 594.768363 / 3.0*2.0; //cy

                cv::Mat Texture32In;
                Texture.convertTo(Texture32In, CV_32F);
                cv::Mat Texture32Out;

                std::cout << Depth.size() << std::endl;

                if (!ReprojectToDepthMap<float>(CameraMatrix, PointCloud, Depth.size(), Depth, Texture32In, Texture32Out, 0.005)){
                    std::cout << "nevyslo to" << std::endl;
                }
                std::cout << Depth.size() << std::endl;

                Depth.convertTo(depthxxx, CV_16U);

                //depthxxx = Depth;
            /*

                std::cout << "TextureOut rows: " << Texture32Out.rows << std::endl;

                Texture32Out.convertTo(Texture, CV_16U);

                sd.m_depthWidth = Depth.cols;
                sd.m_depthHeight = Depth.rows;
                Depth *= 10.0f;
                std::cout << "Depth rows:" << Depth.rows << std::endl;

                sd.m_intrinsic = cv::Matx44f::eye();
                sd.m_extrinsic = cv::Matx44f::eye();

                sd.m_intrinsic(0, 0) = 1824.885927 / 3.0*2.0;
                sd.m_intrinsic(0, 2) = 878.237084 / 3.0*2.0;
                sd.m_intrinsic(1, 1) = 1824.728925 / 3.0*2.0;
                sd.m_intrinsic(1, 2) = 594.768363 / 3.0*2.0;

                RGBDFrame frame;

                cv::Mat texRGB(Texture.rows, Texture.cols, CV_8UC3);
                cv::Mat tex(Texture.rows, Texture.cols, CV_8U);
                Texture.convertTo(tex, CV_8U, 0.25);
                cv::cvtColor(tex, texRGB, CV_GRAY2RGB);
                //std::vector<cv::Mat> vecOfTex(3, tex);
                //cv::merge(vecOfTex, texRGB);
                frame.m_colorCompressed = texRGB.data;
                frame.m_colorSizeBytes = texRGB.size().area() * 3;
                frame.tex = texRGB;

                cv::Mat d16(Depth.rows, Depth.cols, CV_16U);
                Depth.convertTo(d16, CV_16U);
                frame.m_depthCompressed = d16.data;
                frame.m_depthSizeBytes = d16.size().area() * 2;
                frame.depth = d16;

                frame.m_timeStampColor = 0;
                frame.m_timeStampDepth = 0;

                sd.m_frames.push_back(frame);

                cv::imshow("tex", texRGB);
                cv::waitKey(20);
                */
            }

        return true;
    }

    pho::api::PPhoXi JanInit() {

        pho::api::PhoXiFactory Factory;
        //Check if the PhoXi Control Software is running
        if (!Factory.isPhoXiControlRunning()) return nullptr;
        std::cout << "PhoXi Control Software is running" << std::endl;
        //Get List of available devices on the network
        std::vector <pho::api::PhoXiDeviceInformation> DeviceList = Factory.GetDeviceList();
        std::cout << "PhoXi Factory found " << DeviceList.size() << " devices by GetDeviceList call." << std::endl
                  << std::endl;
        for (std::size_t i = 0; i < DeviceList.size(); i++) {
            std::cout << "Device: " << i << std::endl;
            std::cout << "  Name:                    " << DeviceList[i].Name << std::endl;
            std::cout << "  Hardware Identification: " << DeviceList[i].HWIdentification << std::endl;
            std::cout << "  Type:                    " << (std::string) DeviceList[i].Type << std::endl;
            std::cout << "  Firmware version:        " << DeviceList[i].FirmwareVersion << std::endl;
            std::cout << "  Status:                  "
                      << (DeviceList[i].Status.Attached ? "Attached to PhoXi Control. " : "Not Attached to PhoXi Control. ")
                      << (DeviceList[i].Status.Ready ? "Ready to connect" : "Occupied") << std::endl << std::endl;
        }

        //Try to connect Device opened in PhoXi Control, if Any
        pho::api::PPhoXi PhoXiDevice = Factory.CreateAndConnectFirstAttached();
        if (PhoXiDevice) {
            std::cout
                    << "You have already PhoXi device opened in PhoXi Control Software, the API Example is connected to device: "
                    << (std::string) PhoXiDevice->HardwareIdentification << std::endl;
        }
        else {
            std::cout
                    << "You have no PhoXi device opened in PhoXi Control Software, the API Example will try to connect to last device in device list"
                    << std::endl;
            if (!DeviceList.empty()) {
                PhoXiDevice = Factory.CreateAndConnect(DeviceList.back().HWIdentification);
            }
        }
        if (!PhoXiDevice) {
            std::cout << "No device is connected!" << std::endl;
            return nullptr;
        }

        if (PhoXiDevice->isConnected()) {
            std::cout << "Your device is connected" << std::endl;
            if (PhoXiDevice->isAcquiring()) {
                PhoXiDevice->StopAcquisition();
            }
            std::cout << "Starting Software trigger mode" << std::endl;
            PhoXiDevice->TriggerMode = pho::api::PhoXiTriggerMode::Software;
            PhoXiDevice->ClearBuffer();
            PhoXiDevice->StartAcquisition();
            if (PhoXiDevice->isAcquiring()) return PhoXiDevice;
        }
        return nullptr;
    }

    /*void JanTest()
    {
        std::cout << "Start" << std::endl;
        std::cout << sizeof(cv::Matx44f) << std::endl;


        SensorData sd;
        sd.m_sensorName = "Pho";
        sd.m_depthShift = 10000.0f;	//conversion from float[m] to ushort (typically 1000.0f)

        pho::api::PPhoXi PhoXiDevice = JanInit();

        for (size_t i = 0;; i++)
        {
            if (!Scan(PhoXiDevice, i, sd)) break;

        }

        sd.m_frames[0].m_cameraToWorld = cv::Matx44f::eye();

        std::cout << "Saving.." << std::endl;
        sd.saveToFile("pho.sens");

        PhoXiDevice->StopAcquisition();
        PhoXiDevice->Disconnect();
    }*/

    bool grab(cv::Mat& depth)
    {
        pho::api::PFrame frame = scanner->GetSpecificFrame(scanner->TriggerFrame());
        if (((bool)frame) && (frame->Successful))
        {
            std::cout << "cloud size" << frame->PointCloud.Size.Height << std::endl;

            cv::Mat(frame->DepthMap.Size.Height, frame->DepthMap.Size.Width, CV_32FC1, (void*)frame->DepthMap.GetDataPtr()).copyTo(depth);
            depth = depth * 0.001;
        }
        else
        {
            depth.release();
            printf ("no depth\n");
            return false;
        }

        return true;
    }

    KinFuApp() : exit_ (false),  iteractive_mode_(false), pause_(false)
    {
        KinFuParams params = KinFuParams::default_params();
        kinfu_ = KinFu::Ptr( new KinFu(params) );
        pho::api::PhoXiFactory factory;


        //connecting scanner

        lastTimestamp = -1;
        scanner = factory.CreateAndConnect("PhoXiTemp(0)(File3DCamera)",5);

        if(!scanner){
            std::cout << "Cannot connect to scanner" << std::endl;
            exit(-1);
        }
        scanner->TriggerMode =  pho::api::PhoXiTriggerMode::Software;
        scanner->StartAcquisition();

        if(!scanner->isAcquiring()){
            std::cout << "Cannot acquire data" << std::endl;
            exit(-1);
        }


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
        depth.convertTo(display, CV_8U, 255.0/4);
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
//        std::cout << "MAT size" << view_host_.cols << std::endl;
        cv::imshow("Scene", view_host_);
    }

    void take_cloud(KinFu& kinfu)
    {
        cuda::DeviceArray<Point> cloud = kinfu.tsdf().fetchCloud(cloud_buffer);

        cv::Mat cloud_host(1, (int)cloud.size(), CV_32FC4);
        cloud.download(cloud_host.ptr<Point>());
        //viz.showWidget("cloud", cv::viz::WCloud(cloud_host));
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
            //bool has_frame = grab(depth);
            bool has_frame = Scan(depth);

            if (!has_frame){
                return std::cout << "Can't grab depth" << std::endl, false;
            }


            depth_device_.upload(depth.data, depth.step, depth.rows, depth.cols);
            {
                SampledScopeTime fps(time_ms); (void)fps;
                //has_image = kinfu(depth_device_);
                has_image = kinfu(depth_device_);
            }

            // reading cloud data
            /*ply_set_read_cb(ply, "vertex", "x", this->vertex_X, NULL, 0);
            ply_set_read_cb(ply, "vertex", "y", this->vertex_Y, NULL, 0);
            ply_set_read_cb(ply, "vertex", "z", this->vertex_Z, NULL, 1);*/

            if (has_image)
                show_raycasted(kinfu);

            show_depth(depth);
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

    bool pause_ /*= false*/;
    bool exit_, iteractive_mode_;
   // OpenNISource& capture_;
    KinFu::Ptr kinfu_;
    cv::viz::Viz3d viz;

    cv::Mat view_host_;
    cuda::Image view_device_;
    cuda::Depth depth_device_;
    cuda::Cloud cloud_device_;
    cuda::DeviceArray<Point> cloud_buffer;
    double lastTimestamp;
    pho::api::PPhoXi scanner;
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
