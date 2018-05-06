// PhoXiAPIReference.cpp : Defines the entry point for the console application.
//



//#define PHOXI_EXAMPLES

#include "Geometry/Volumetric/VolumeWarperPrism.h"
#include "Geometry/SphereFitter.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/PCLPointCloud2.h>
#include <opencv2/opencv.hpp>
#include "Console/PhotoneoConsole.h" 
#include "NewCamera/Camera.h"
#include "PhoInputOutput/Phoio.h"
#include "TypesRegistrator.h"
#include "boost/thread/thread.hpp"
#include "PhoInputOutput/Phoio.h"


#include <Geometry/Transformation3D.h>

#include "CameraCalibration/PhotoneoCameraCalibrationTool.h"

#define PHOXI_PCL_SUPPORT
#define PHOXI_OPENCV_SUPPORT

#include "../PhoXi.h"
#include "../PhoXiRawAccess.h"
#include <string>
#if defined(_WIN32)
#include <windows.h>
#elif defined (__linux__)
#include <unistd.h>
#endif



//////////////////////////////////////////////////////////////////////////
//  Photoneo's C++ API Example
//     This example is divided into multiple member functions of
//      PhoXiExamples object
//     If you have any questions, write us at support@photoneo.com
//////////////////////////////////////////////////////////////////////////

#if defined(_WIN32)
#define LOCAL_CROSS_SLEEP(Millis) Sleep(Millis)
#elif defined (__linux__) || defined(__APPLE__)
#define LOCAL_CROSS_SLEEP(Millis) usleep(Millis * 1000)
#endif

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "glog/logging.h"


#include "SerialCommunication/SimpleAxis.h"

#ifdef WIN32
#   include "Thorlabs.MotionControl.IntegratedStepperMotors.h"
#endif

#ifdef PHOXI_EXAMPLES

//The whole api is in namespace pho (Photoneo) :: api
class PhoXiExamples {
private:
	template<class T>
	bool ReadLine(T &Output) const {
		std::string Input;
		std::getline(std::cin, Input);
		std::stringstream InputSteam(Input);
		if (InputSteam >> Output) {
			return true;
		}
		else {
			return false;
		}
	}
	bool ReadLine(std::string &Output) const {
		std::getline(std::cin, Output);
		return true;
	}
public:
	std::vector <pho::api::PhoXiDeviceInformation> DeviceList;
	pho::api::PPhoXi PhoXiDevice;
	pho::api::PhoXiFactory Factory;
	pho::api::PFrame SampleFrame;
	void GetAvailableDevicesExample() {
		//Wait for the PhoXiControl
		while (!Factory.isPhoXiControlRunning()) {
			LOCAL_CROSS_SLEEP(100);
		}
		std::cout << "PhoXi Control Version: " << Factory.GetPhoXiControlVersion() << std::endl;
		std::cout << "PhoXi API Version: " << Factory.GetAPIVersion() << std::endl;
		DeviceList = Factory.GetDeviceList();
		std::cout << "PhoXi Factory found " << DeviceList.size() << " devices by GetDeviceList call." << std::endl
			<< std::endl;
		for (std::size_t i = 0; i < DeviceList.size(); i++) {
			std::cout << "Device: " << i << std::endl;
			std::cout << "  Name:                    " << DeviceList[i].Name << std::endl;
			std::cout << "  Hardware Identification: " << DeviceList[i].HWIdentification << std::endl;
			std::cout << "  Type:                    " << (std::string) DeviceList[i].Type << std::endl;
			std::cout << "  Firmware version:        " << DeviceList[i].FirmwareVersion << std::endl;
			std::cout << "  Status:                  " << (DeviceList[i].Status.Attached ? "Attached to PhoXi Control. "
				: "Not Attached to PhoXi Control. ")
				<< (DeviceList[i].Status.Ready ? "Ready to connect" : "Occupied") << std::endl << std::endl;
		}
	}
	void ConnectPhoXiDeviceExample() {
		//You can connect to any device connected to local network (with compatible ip4 settings)
		//The connection can be made in multiple ways
		while (true) {
			std::cout << "Please enter the number of the way to connect to your device from this possibilities:"
				<< std::endl;
			std::cout << "  1. Connect by Hardware Identification Number" << std::endl;
			std::cout << "  2. Connect by Index listed from GetDeviceList call" << std::endl;
			std::cout << "  3. Connect first device Attached to PhoXi Control - if Any" << std::endl << std::endl;
			std::cout << "  4. Refresh GetDeviceList" << std::endl << std::endl;
			std::cout << "Please enter the choice: ";
			std::size_t Index;
			if (!ReadLine(Index)) continue;
			switch (Index) {
			case 1:
				ConnectPhoXiDeviceBySerialExample();
				break;
			case 2:
				ConnectPhoXiDeviceByPhoXiDeviceInformationEntryExample();
				break;
			case 3:
				ConnectFirstAttachedPhoXiDeviceExample();
				break;
			case 4:
				GetAvailableDevicesExample();
				break;
			default:
				continue;
			}
			if (PhoXiDevice && PhoXiDevice->isConnected()) break;
		}
	}
	void ConnectPhoXiDeviceBySerialExample() {
		std::cout << std::endl << "Please enter the Hardware Identification Number: ";
		std::string HardwareIdentification;
		if (!ReadLine(HardwareIdentification)) return;
		pho::api::PhoXiTimeout Timeout = pho::api::PhoXiTimeout::ZeroTimeout;
		PhoXiDevice = Factory.CreateAndConnect(HardwareIdentification, Timeout);
		if (PhoXiDevice) {
			std::cout << "Connection to the device " << HardwareIdentification << " was Successful!" << std::endl;
		}
		else {
			std::cout << "Connection to the device " << HardwareIdentification << " was Unsuccessful!" << std::endl;
		}
	}
	void ConnectPhoXiDeviceByPhoXiDeviceInformationEntryExample() {
		std::cout << std::endl << "Please enter the Index listed from GetDeviceList call: ";
		std::size_t Index;
		if (!ReadLine(Index)) return;
		if (Index >= DeviceList.size()) {
			std::cout << "Bad Index, or not number!" << std::endl;
			return;
		}
		PhoXiDevice = Factory.Create(DeviceList[Index]);
		if (PhoXiDevice) {
			if (PhoXiDevice->Connect()) {
				std::cout << "Connection to the device " << DeviceList[Index].HWIdentification << " was Successful!"
					<< std::endl;
			}
			else {
				std::cout << "Connection to the device " << DeviceList[Index].HWIdentification << " was Unsuccessful!"
					<< std::endl;
			}
		}
		else {
			std::cout << "Unspecified error" << std::endl;
		}
	}
	void ConnectFirstAttachedPhoXiDeviceExample() {
		PhoXiDevice = Factory.CreateAndConnectFirstAttached();
		if (PhoXiDevice) {
			std::cout << "Connection to the device " << (std::string) PhoXiDevice->HardwareIdentification
				<< " was Successful!" << std::endl;
		}
		else {
			std::cout << "There is no attached device, or the device is not ready!" << std::endl;
		}
	}
	void BasicDeviceStateExample() {
		//Check if the device is connected
		if (PhoXiDevice && PhoXiDevice->isConnected()) {
			std::cout << "You are connected to " << (std::string) PhoXiDevice->GetType()
				<< " with Hardware Identification " << (std::string) PhoXiDevice->HardwareIdentification
				<< std::endl;
			std::vector <std::string> SupportedFeatures = PhoXiDevice->Features.GetSupportedFeatures();
			std::cout << "  Status:" << std::endl;
			std::cout << "    "
				<< (PhoXiDevice->isConnected() ? "Device is connected" : "Device is not connected (Error)")
				<< std::endl;
			std::cout << "    " << (PhoXiDevice->isAcquiring() ? "Device is in acquisition mode"
				: "Device is not in acquisition mode") << std::endl;
			std::cout << "  This device have these features supported:";
			for (std::size_t i = 0; i < SupportedFeatures.size(); i++) {
				std::cout << " " << SupportedFeatures[i] << ";";
			}
			std::cout << std::endl << std::endl;
			//We will go trough all current Device features
			//You can ask the feature if it is implemented and if it is possible to Get or Set the feature value
			if (PhoXiDevice->CapturingMode.isEnabled() && PhoXiDevice->CapturingMode.CanGet()) {
				pho::api::PhoXiCapturingMode CapturingMode = PhoXiDevice->CapturingMode;
				//You can ask the feature, if the last performed operation was successful
				if (!PhoXiDevice->CapturingMode.isLastOperationSuccessful()) throw std::runtime_error(PhoXiDevice->CapturingMode.GetLastErrorMessage().c_str());
				pho::api::PhoXiSize Resolution = CapturingMode.Resolution;
				//you can also access the resolution by PhoXiDevice->Resolution;
				std::cout << "  CapturingMode: " << std::endl;
				std::cout << "    Resolution:" << std::endl;
				std::cout << "      Width: " << Resolution.Width << std::endl;
				std::cout << "      Height: "
					<< PhoXiDevice->Resolution->Height /*You can also directly access the value inside*/
					<< std::endl;
			}
			if (PhoXiDevice->CapturingSettings.isEnabled() && PhoXiDevice->CapturingSettings.CanGet()) {
				pho::api::PhoXiCapturingSettings CapturingSettings = PhoXiDevice->CapturingSettings;
				if (!PhoXiDevice->CapturingSettings.isLastOperationSuccessful()) throw std::runtime_error(PhoXiDevice->CapturingSettings.GetLastErrorMessage().c_str());
				std::cout << "  CapturingSettings: " << std::endl;
				std::cout << "    ShutterMultiplier: " << CapturingSettings.ShutterMultiplier << std::endl;
				std::cout << "    ScanMultiplier: " << CapturingSettings.ScanMultiplier << std::endl;
			}
			if (PhoXiDevice->TriggerMode.isEnabled() && PhoXiDevice->TriggerMode.CanGet()) {
				pho::api::PhoXiTriggerMode TriggerMode = PhoXiDevice->TriggerMode;
				if (!PhoXiDevice->TriggerMode.isLastOperationSuccessful()) throw std::runtime_error(PhoXiDevice->TriggerMode.GetLastErrorMessage().c_str());
				std::cout << "  TriggerMode: " << (std::string) TriggerMode << std::endl;
			}
			if (PhoXiDevice->Timeout.isEnabled() && PhoXiDevice->Timeout.CanGet()) {
				pho::api::PhoXiTimeout Timeout = PhoXiDevice->Timeout;
				if (!PhoXiDevice->Timeout.isLastOperationSuccessful()) throw std::runtime_error(PhoXiDevice->Timeout.GetLastErrorMessage().c_str());
				std::cout << "  Timeout: " << (std::string) Timeout << std::endl;
			}
			if (PhoXiDevice->ProcessingSettings.isEnabled() && PhoXiDevice->ProcessingSettings.CanGet()) {
				pho::api::PhoXiProcessingSettings ProcessingSettings = PhoXiDevice->ProcessingSettings;
				if (!PhoXiDevice->ProcessingSettings.isLastOperationSuccessful()) throw std::runtime_error(PhoXiDevice->ProcessingSettings.GetLastErrorMessage().c_str());
				std::cout << "  ProcessingSettings: " << std::endl;
				std::cout << "    Confidence: " << ProcessingSettings.Confidence << std::endl;
			}
			if (PhoXiDevice->OutputSettings.isEnabled() && PhoXiDevice->OutputSettings.CanGet()) {
				pho::api::FrameOutputSettings OutputSettings = PhoXiDevice->OutputSettings;
				if (!PhoXiDevice->OutputSettings.isLastOperationSuccessful()) throw std::runtime_error(PhoXiDevice->OutputSettings.GetLastErrorMessage().c_str());
				std::cout << "  OutputSettings: " << std::endl;
				std::cout << "    SendConfidenceMap: " << (OutputSettings.SendConfidenceMap ? "Yes" : "No")
					<< std::endl;
				std::cout << "    SendDepthMap: " << (OutputSettings.SendDepthMap ? "Yes" : "No") << std::endl;
				std::cout << "    SendNormalMap: " << (OutputSettings.SendNormalMap ? "Yes" : "No") << std::endl;
				std::cout << "    SendPointCloud: " << (OutputSettings.SendPointCloud ? "Yes" : "No") << std::endl;
				std::cout << "    SendTexture: " << (OutputSettings.SendTexture ? "Yes" : "No") << std::endl;
			}
			if (PhoXiDevice->SupportedCapturingModes.isEnabled() && PhoXiDevice->SupportedCapturingModes.CanGet()) {
				std::vector <pho::api::PhoXiCapturingMode>
					SupportedCapturingModes = PhoXiDevice->SupportedCapturingModes;
				if (!PhoXiDevice->SupportedCapturingModes.isLastOperationSuccessful())
					throw std::runtime_error(PhoXiDevice->SupportedCapturingModes.GetLastErrorMessage().c_str());
				std::cout << "  SupportedCapturingModes: " << std::endl;
				for (std::size_t i = 0; i < SupportedCapturingModes.size(); i++) {
					std::cout << "    (" << std::to_string(SupportedCapturingModes[i].Resolution.Width) << "x"
						<< std::to_string(SupportedCapturingModes[i].Resolution.Height) << ")" << std::endl;
				}
			}
			if (PhoXiDevice->HardwareIdentification.isEnabled() && PhoXiDevice->HardwareIdentification.CanGet()) {
				std::string HardwareIdentification = PhoXiDevice->HardwareIdentification;
				if (!PhoXiDevice->HardwareIdentification.isLastOperationSuccessful())
					throw std::runtime_error(PhoXiDevice->HardwareIdentification.GetLastErrorMessage().c_str());
				std::cout << "  HardwareIdentification: " << HardwareIdentification << std::endl;
			}
		}
	}
	void FreerunExample() {
		//Check if the device is connected
		if (PhoXiDevice && PhoXiDevice->isConnected()) {
			//If it is not in Freerun mode, we need to switch the modes
			if (PhoXiDevice->TriggerMode != pho::api::PhoXiTriggerMode::Freerun) {
				std::cout << "Device is not in Freerun mode" << std::endl;
				if (PhoXiDevice->isAcquiring()) {
					std::cout << "Stopping acquisition" << std::endl;
					//If the device is in Acquisition mode, we need to stop the acquisition
					if (!PhoXiDevice->StopAcquisition()) {
						throw std::runtime_error("Error in StopAcquistion");
					}
				}
				std::cout << "Switching to Freerun mode " << std::endl;
				//Switching the mode is as easy as assigning of a value, it will call the appropriate calls in the background
				PhoXiDevice->TriggerMode = pho::api::PhoXiTriggerMode::Freerun;
				//Just check if did everything run smoothly
				if (!PhoXiDevice->TriggerMode.isLastOperationSuccessful()) throw std::runtime_error(PhoXiDevice->TriggerMode.GetLastErrorMessage().c_str());
			}
			//Start the device acquisition, if necessary
			if (!PhoXiDevice->isAcquiring()) {
				if (!PhoXiDevice->StartAcquisition()) {
					throw std::runtime_error("Error in StartAcquisition");
				}
			}
			//We can clear the current Acquisition buffer -- This will not clear Frames that arrives to the PC after the Clear command is performed
			int ClearedFrames = PhoXiDevice->ClearBuffer();
			std::cout << ClearedFrames << " were cleared from the cyclic buffer" << std::endl;

			//While we checked the state of the StartAcquisition call, this check is not necessary, but it is a good practice
			if (PhoXiDevice->isAcquiring()) {
				for (std::size_t i = 0; i < 5; i++) {
					std::cout << "Waiting for frame " << i << std::endl;
					//Get the frame
					pho::api::PFrame Frame =
						PhoXiDevice->GetFrame(/*You can specify Timeout here - default is the Timeout stored in Timeout Feature -> Infinity by default*/);
					if (Frame) {
						std::cout << "Frame retrieved" << std::endl;
						std::cout << "  Frame params: " << std::endl;
						std::cout << "    Frame Index: " << Frame->Info.FrameIndex << std::endl;
						std::cout << "    Frame Timestamp: " << Frame->Info.FrameTimestamp << std::endl;
						std::cout << "    Frame Duration: " << Frame->Info.FrameDuration << std::endl;
						std::cout << "    Frame Resolution: " << Frame->GetResolution().Width << " x "
							<< Frame->GetResolution().Height << std::endl;
						std::cout << "    Sensor Position: " << Frame->Info.SensorPosition.x << "; "
							<< Frame->Info.SensorPosition.y << "; " << Frame->Info.SensorPosition.z << std::endl;
						if (!Frame->Empty()) {
							std::cout << "  Frame data: " << std::endl;
							if (!Frame->PointCloud.Empty()) {
								std::cout << "    PointCloud: " << Frame->PointCloud.Size.Width << " x "
									<< Frame->PointCloud.Size.Height << " Type: "
									<< Frame->PointCloud.GetElementName() << std::endl;
							}
							if (!Frame->NormalMap.Empty()) {
								std::cout << "    NormalMap: " << Frame->NormalMap.Size.Width << " x "
									<< Frame->NormalMap.Size.Height << " Type: "
									<< Frame->NormalMap.GetElementName() << std::endl;
							}
							if (!Frame->DepthMap.Empty()) {
								std::cout << "    DepthMap: " << Frame->DepthMap.Size.Width << " x "
									<< Frame->DepthMap.Size.Height << " Type: "
									<< Frame->DepthMap.GetElementName() << std::endl;
							}
							if (!Frame->ConfidenceMap.Empty()) {
								std::cout << "    ConfidenceMap: " << Frame->ConfidenceMap.Size.Width << " x "
									<< Frame->ConfidenceMap.Size.Height << " Type: "
									<< Frame->ConfidenceMap.GetElementName() << std::endl;
							}
							if (!Frame->Texture.Empty()) {
								std::cout << "    Texture: " << Frame->Texture.Size.Width << " x "
									<< Frame->Texture.Size.Height << " Type: " << Frame->Texture.GetElementName()
									<< std::endl;
							}
						}
						else {
							std::cout << "Frame is empty.";
						}
					}
					else {
						std::cout << "Failed to retrieve the frame!";
					}
				}
			}
		}
	}
	void SoftwareTriggerExample() {
		//Check if the device is connected
		if (PhoXiDevice && PhoXiDevice->isConnected()) {
			//If it is not in Software trigger mode, we need to switch the modes
			if (PhoXiDevice->TriggerMode != pho::api::PhoXiTriggerMode::Software) {
				std::cout << "Device is not in Software trigger mode" << std::endl;
				if (PhoXiDevice->isAcquiring()) {
					std::cout << "Stopping acquisition" << std::endl;
					//If the device is in Acquisition mode, we need to stop the acquisition
					if (!PhoXiDevice->StopAcquisition()) {
						throw std::runtime_error("Error in StopAcquistion");
					}
				}
				std::cout << "Switching to Software trigger mode " << std::endl;
				//Switching the mode is as easy as assigning of a value, it will call the appropriate calls in the background
				PhoXiDevice->TriggerMode = pho::api::PhoXiTriggerMode::Software;
				//Just check if did everything run smoothly
				if (!PhoXiDevice->TriggerMode.isLastOperationSuccessful()) throw std::runtime_error(PhoXiDevice->TriggerMode.GetLastErrorMessage().c_str());
			}
			//Start the device acquisition, if necessary
			if (!PhoXiDevice->isAcquiring()) {
				if (!PhoXiDevice->StartAcquisition()) {
					throw std::runtime_error("Error in StartAcquisition");
				}
			}
			//We can clear the current Acquisition buffer -- This will not clear Frames that arrives to the PC after the Clear command is performed
			int ClearedFrames = PhoXiDevice->ClearBuffer();
			std::cout << ClearedFrames << " frames were cleared from the cyclic buffer" << std::endl;

			//While we checked the state of the StartAcquisition call, this check is not necessary, but it is a good practice
			if (PhoXiDevice->isAcquiring()) {
				for (std::size_t i = 0; i < 5; i++) {
					std::cout << "Triggering the " << i << "-th frame" << std::endl;
					int FrameID =
						PhoXiDevice->TriggerFrame(/*If false is passed here, the device will reject the frame if it is not ready to be triggered, if true us supplied, it will wait for the trigger*/);
					if (FrameID < 0) {
						//If negative number is returned trigger was unsuccessful
						std::cout << "Trigger was unsuccessful!" << std::endl;
						continue;
					}
					else {
						std::cout << "Frame was triggered, Frame Id: " << FrameID << std::endl;
					}
					std::cout << "Waiting for frame " << i << std::endl;
					//Wait for a frame with specific FrameID. There is a possibility, that frame triggered before the trigger will arrive after the trigger call, and will be retrieved before requested frame
					//  Because of this, the TriggerFrame call returns the requested frame ID, so it can than be retrieved from the Frame structure. This call is doing that internally in background
					pho::api::PFrame Frame =
						PhoXiDevice->GetSpecificFrame(FrameID/*, You can specify Timeout here - default is the Timeout stored in Timeout Feature -> Infinity by default*/);
					if (Frame) {
						std::cout << "Frame retrieved" << std::endl;
						std::cout << "  Frame params: " << std::endl;
						std::cout << "    Frame Index: " << Frame->Info.FrameIndex << std::endl;
						std::cout << "    Frame Timestamp: " << Frame->Info.FrameTimestamp << std::endl;
						std::cout << "    Frame Duration: " << Frame->Info.FrameDuration << std::endl;
						std::cout << "    Frame Resolution: " << Frame->GetResolution().Width << " x "
							<< Frame->GetResolution().Height << std::endl;
						std::cout << "    Sensor Position: " << Frame->Info.SensorPosition.x << "; "
							<< Frame->Info.SensorPosition.y << "; " << Frame->Info.SensorPosition.z << std::endl;
						if (!Frame->Empty()) {
							std::cout << "  Frame data: " << std::endl;
							if (!Frame->PointCloud.Empty()) {
								std::cout << "    PointCloud: " << Frame->PointCloud.Size.Width << " x "
									<< Frame->PointCloud.Size.Height << " Type: "
									<< Frame->PointCloud.GetElementName() << std::endl;
							}
							if (!Frame->NormalMap.Empty()) {
								std::cout << "    NormalMap: " << Frame->NormalMap.Size.Width << " x "
									<< Frame->NormalMap.Size.Height << " Type: "
									<< Frame->NormalMap.GetElementName() << std::endl;
							}
							if (!Frame->DepthMap.Empty()) {
								std::cout << "    DepthMap: " << Frame->DepthMap.Size.Width << " x "
									<< Frame->DepthMap.Size.Height << " Type: "
									<< Frame->DepthMap.GetElementName() << std::endl;
							}
							if (!Frame->ConfidenceMap.Empty()) {
								std::cout << "    ConfidenceMap: " << Frame->ConfidenceMap.Size.Width << " x "
									<< Frame->ConfidenceMap.Size.Height << " Type: "
									<< Frame->ConfidenceMap.GetElementName() << std::endl;
							}
							if (!Frame->Texture.Empty()) {
								std::cout << "    Texture: " << Frame->Texture.Size.Width << " x "
									<< Frame->Texture.Size.Height << " Type: " << Frame->Texture.GetElementName()
									<< std::endl;
							}
						}
						else {
							std::cout << "Frame is empty.";
						}
					}
					else {
						std::cout << "Failed to retrieve the frame!";
					}
				}
			}
		}
	}
	void ChangeSettingsExample() {
		//Check if the device is connected
		if (PhoXiDevice && PhoXiDevice->isConnected()) {
			//Check if the feature is supported and if it we have required access permissions
			//  These checks are not necessary, these have in mind multiple different devices in the future
			if (!PhoXiDevice->CapturingSettings.isEnabled() || !PhoXiDevice->CapturingSettings.CanSet()
				|| !PhoXiDevice->CapturingSettings.CanGet()) {
				std::cout
					<< "Settings used in example are not supported by the Device Hardware, or are Read only on the specific device"
					<< std::endl;
				return;
			}
			std::cout << "Settings change example" << std::endl;

			//For purpose of this example, we will change the trigger mode to Software Trigger, it is not necessary for the exhibition of desired functionality

			if (PhoXiDevice->TriggerMode != pho::api::PhoXiTriggerMode::Software) {
				if (PhoXiDevice->isAcquiring()) {
					if (!PhoXiDevice->StopAcquisition()) {
						throw std::runtime_error("Error in StopAcquistion");
					}
				}
				PhoXiDevice->TriggerMode = pho::api::PhoXiTriggerMode::Software;
				//Just check if did everything run smoothly
				if (!PhoXiDevice->TriggerMode.isLastOperationSuccessful()) throw std::runtime_error(PhoXiDevice->TriggerMode.GetLastErrorMessage().c_str());
			}
			if (!PhoXiDevice->isAcquiring()) PhoXiDevice->StartAcquisition();

			int CurrentShutterMultiplier = PhoXiDevice->CapturingSettings->ShutterMultiplier;

			//To change the setting, just assign a new value
			PhoXiDevice->CapturingSettings->ShutterMultiplier = CurrentShutterMultiplier + 1;

			//You can check if the operation succeed
			if (!PhoXiDevice->CapturingSettings.isLastOperationSuccessful()) throw std::runtime_error(PhoXiDevice->CapturingSettings.GetLastErrorMessage().c_str());

			//Get the current Output configuration
			pho::api::FrameOutputSettings CurrentOutputSettings = PhoXiDevice->OutputSettings;
			pho::api::FrameOutputSettings NewOutputSettings = CurrentOutputSettings;
			NewOutputSettings.SendPointCloud = true;
			NewOutputSettings.SendNormalMap = true;
			NewOutputSettings.SendDepthMap = true;
			NewOutputSettings.SendConfidenceMap = true;
			NewOutputSettings.SendTexture = true;
			//Send all outputs
			PhoXiDevice->OutputSettings = NewOutputSettings;

			//Trigger the frame
			int FrameID = PhoXiDevice->TriggerFrame();
			//Check if the frame was successfully triggered
			if (FrameID < 0) throw std::runtime_error("Software trigger failed!");
			//Retrieve the frame
			pho::api::PFrame Frame = PhoXiDevice->GetFrame(FrameID);
			if (Frame) {
				//Save the frame for next example
				SampleFrame = Frame;
			}

			//Change the setting back
			PhoXiDevice->OutputSettings = CurrentOutputSettings;
			PhoXiDevice->CapturingSettings->ShutterMultiplier = CurrentShutterMultiplier;

			if (!PhoXiDevice->CapturingSettings.isLastOperationSuccessful()) throw std::runtime_error(PhoXiDevice->CapturingSettings.GetLastErrorMessage().c_str());

			//Try to change device resolution
			if (PhoXiDevice->SupportedCapturingModes.isEnabled() && PhoXiDevice->SupportedCapturingModes.CanGet()
				&& PhoXiDevice->CapturingMode.isEnabled() && PhoXiDevice->CapturingMode.CanSet()
				&& PhoXiDevice->CapturingMode.CanGet()) {
				//Retrieve current capturing mode
				pho::api::PhoXiCapturingMode CurrentCapturingMode = PhoXiDevice->CapturingMode;
				if (!PhoXiDevice->CapturingMode.isLastOperationSuccessful()) throw std::runtime_error(PhoXiDevice->CapturingMode.GetLastErrorMessage().c_str());

				//Get all supported modes
				std::vector <pho::api::PhoXiCapturingMode>
					SupportedCapturingModes = PhoXiDevice->SupportedCapturingModes;
				if (!PhoXiDevice->SupportedCapturingModes.isLastOperationSuccessful())
					throw std::runtime_error(PhoXiDevice->SupportedCapturingModes.GetLastErrorMessage().c_str());

				//Cycle trough all other Supported modes, change the settings and grab a frame
				for (std::size_t i = 0; i < SupportedCapturingModes.size(); i++) {
					if (!(SupportedCapturingModes[i] == CurrentCapturingMode)) {
						PhoXiDevice->CapturingMode = SupportedCapturingModes[i];
						if (!PhoXiDevice->CapturingMode.isLastOperationSuccessful())
							throw std::runtime_error(PhoXiDevice->CapturingMode.GetLastErrorMessage().c_str());
						//Trigger Frame
						int FrameID = PhoXiDevice->TriggerFrame();
						if (FrameID < 0) throw std::runtime_error("Software trigger failed!");
						Frame = PhoXiDevice->GetSpecificFrame(FrameID);
						if (Frame) {
							std::cout << "Arrived Frame Resolution: " << Frame->GetResolution().Width << " x "
								<< Frame->GetResolution().Height << std::endl;
						}
					}
				}
				//Change the mode back
				PhoXiDevice->CapturingMode = CurrentCapturingMode;
				if (!PhoXiDevice->CapturingMode.isLastOperationSuccessful()) throw std::runtime_error(PhoXiDevice->CapturingMode.GetLastErrorMessage().c_str());

			}
		}
	}
	void DataHandlingExample() {
		//Check if we have SampleFrame Data
		if (SampleFrame && !SampleFrame->Empty()) {
			//We will count the number of measured points
			if (!SampleFrame->PointCloud.Empty()) {
				int MeasuredPoints = 0;
				pho::api::Point3_32f ZeroPoint(0.0f, 0.0f, 0.0f);
				for (int y = 0; y < SampleFrame->PointCloud.Size.Height; y++) {
					for (int x = 0; x < SampleFrame->PointCloud.Size.Width; x++) {
						if (SampleFrame->PointCloud[y][x] != ZeroPoint) {
							MeasuredPoints++;
						}
					}
				}
				std::cout << "Your sample Point cloud has " << MeasuredPoints << " measured points." << std::endl;
				pho::api::Point3_32f *RawPointer = SampleFrame->PointCloud.GetDataPtr();
				float *MyLocalCopy = new float[SampleFrame->PointCloud.GetElementsCount() * 3];
				memcpy(MyLocalCopy, RawPointer, SampleFrame->PointCloud.GetDataSize());
				//Data are organized as a matrix of X, Y, Z floats, see the documentation for all other types
				delete[] MyLocalCopy;
				//Data from SampleFrame, or all other frames that are returned by the device are copied from the Cyclic buffer and will remain in the memory until the Frame will go out of scope
				//You can specifically call SampleFrame->PointCloud.Clear() to release some of the data
			}
			//You can store the Frame as a ply structure
			SampleFrame->SaveAsPly("SampleFrame.ply"/*, You have multiple storing options*/);
			//If you want OpenCV support, you need to link appropriate libraries and add OpenCV include directory
			//To add the support, add #define PHOXI_OPENCV_SUPPORT before include of PhoXi include files
#ifdef PHOXI_OPENCV_SUPPORT
			if (!SampleFrame->PointCloud.Empty()) {
				cv::Mat PointCloudMat;
				if (SampleFrame->PointCloud.ConvertTo(PointCloudMat)) {
					cv::Point3f MiddlePoint = PointCloudMat.at<cv::Point3f>(PointCloudMat.rows, PointCloudMat.cols);
					std::cout << "Middle point: " << MiddlePoint.x << "; " << MiddlePoint.y << "; " << MiddlePoint.z;
				}
			}
#endif
			//If you want PCL support, you need to link appropriate libraries and add PCL include directory
			//To add the support, add #define PHOXI_PCL_SUPPORT before include of PhoXi include files
#ifdef PHOXI_PCL_SUPPORT
			//The PCL convert will convert the appropriate data into the pcl PointCloud based on the Point Cloud type
			pcl::PointCloud <pcl::PointXYZRGBNormal> MyPCLCloud;
			SampleFrame->ConvertTo(MyPCLCloud);
#endif
		}
	}
	void CorrectDisconnectExample() {
		//The whole API is designed on C++ standards, using smart pointers and constructor/destructor logic
		//All resources will be closed automatically, but the device state will not be affected -> it will remain connected in PhoXi Control and if in freerun, it will remain Scanning
		//To Stop the device, just
		PhoXiDevice->StopAcquisition();
		//If you want to disconnect and logout the device from PhoXi Control, so it will then be available for other devices, call
		std::cout << "Do you want to logout the device? Enter 0 for no, enter 1 for yes: ";
		bool Entry;
		if (!ReadLine(Entry)) return;
		PhoXiDevice->Disconnect(Entry);
		//The call PhoXiDevice without Logout will be called automatically by destructor
	}
	PhoXiExamples() {
		try {
			GetAvailableDevicesExample();
			ConnectPhoXiDeviceExample();
			for (std::size_t i = 0; i < 1; i++) {
				BasicDeviceStateExample();
				FreerunExample();
				SoftwareTriggerExample();
				ChangeSettingsExample();
				DataHandlingExample();
			}
			CorrectDisconnectExample();
		}
		catch (std::exception &InternalException) {
			std::cout << std::endl << "Exception was thrown: " << InternalException.what() << std::endl;
		}
	}
};
#endif
#if defined(_WIN32)
#define LOCAL_CROSS_SLEEP(Millis) Sleep(Millis)
#elif defined (__linux__) || defined(__APPLE__)
#define LOCAL_CROSS_SLEEP(Millis) usleep(Millis * 1000)
#endif

#define SweepUpTimePath "Global/Projector/Settings/SweepUpTime.AHBox<double>"
#define SweepDownTimePath "Global/Projector/Settings/SweepDownTime.AHBox<double>"
#define IndexMapPath "Actions/BasicDepthAction/0/index.Mat"
#define UseCalibratedIndexOffsetsPath "Global/Projector/Settings/UseCalibratedIndexOffsets.AHBox<bool>"

cv::Mat CaptureAndGetIndexMap(pho::api::PPhoXi Scanner) {
	pho::api::PhoXiRawAccessHandler RawAccessHandler(Scanner);
	int SWTriggerID = Scanner->TriggerImage();
	if (SWTriggerID < 0) return cv::Mat();

	cv::Mat IndexMap;

	do {
		pho::api::PFrame Frame = Scanner->GetFrame();
		if (Frame) {
			if (Frame->Info.FrameIndex == SWTriggerID) {
				pho::PDataManager WholeDM = RawAccessHandler.GetLastOutput();
				if (WholeDM) {
					IndexMap = WholeDM->GetLeaf(IndexMapPath).Ref<cv::Mat>();
					return IndexMap;
				}
			}
			else {
				if (Frame->Info.FrameIndex > SWTriggerID) return cv::Mat();
			}
		}
	} while (true);
}

bool FindClusterDiff(const std::vector<double>& Diffs, double& Result) {
	if (Diffs.size() < 10) return false;
	const float ClusterWidth = 1.0f;
	std::size_t ClusterBegin = 0;
	std::size_t ClusterEnd = 0;
	double ClusterSum = 0.0;
	double ClusterMedian;
	double ClusterMean;
	std::size_t ClusterSize;

	std::size_t MaxClusterSize = 0;
	std::size_t MaxClusterBegin, MaxClusterEnd;
	double MaxClusterMean = -1000.0;
	double MaxClusterMedian;

	std::vector<double> ClusterMeans;
	std::vector <std::size_t> ClusterSizes;

	while (ClusterEnd < Diffs.size()
		&& Diffs[ClusterEnd] - Diffs[ClusterBegin] <= ClusterWidth) {
		ClusterSum += Diffs[ClusterEnd];
		ClusterEnd++;
	}
	ClusterMedian = Diffs[(ClusterBegin + ClusterEnd) / 2];
	ClusterMean = ClusterSum / double(ClusterEnd - ClusterBegin);
	ClusterSize = ClusterEnd - ClusterBegin;
	MaxClusterSize = ClusterSize;
	MaxClusterBegin = ClusterBegin;
	MaxClusterEnd = ClusterEnd;
	MaxClusterMedian = ClusterMedian;
	MaxClusterMean = ClusterMean;
	ClusterMeans.push_back(ClusterMean);
	ClusterSizes.push_back(ClusterSize);

	for (ClusterEnd++; ClusterEnd <= Diffs.size(); ClusterEnd++) {
		ClusterSum += Diffs[ClusterEnd - 1];
		while (Diffs[ClusterEnd - 1] - Diffs[ClusterBegin] > ClusterWidth) {
			ClusterSum -= Diffs[ClusterBegin];
			ClusterBegin++;
		}
		ClusterMedian = Diffs[(ClusterBegin + ClusterEnd) / 2];
		ClusterMean = ClusterSum / double(ClusterEnd - ClusterBegin);
		ClusterSize = ClusterEnd - ClusterBegin;
		ClusterMeans.push_back(ClusterMean);
		ClusterSizes.push_back(ClusterSize);
		if (ClusterEnd - ClusterBegin > MaxClusterSize) {
			MaxClusterSize = ClusterSize;
			MaxClusterBegin = ClusterBegin;
			MaxClusterEnd = ClusterEnd;
			MaxClusterMedian = ClusterMedian;
			MaxClusterMean = ClusterMean;
		}
	}
	Result = MaxClusterMean;
	return Result != -1000.0;
}

void CalibrateOffsetShifts(const std::vector<double> &SupportedSweepUpTimes,
	const std::string &ChoosenDeviceID = "",
	const std::vector <std::string> &ImageFilePaths = std::vector<std::string>(),
	const int DefaultSweepUpTimeIndex = 0) {
	bool FromFiles = ImageFilePaths.size() > 0;
	const float Tolerance = 0.2f;
	const float ClusterWidth = 1.0f;
	const float IndexMin = 0.0;
	const float IndexMax = 511.0;

	double DefaultSweepUpTime;
	pho::api::PhoXiFactory Factory;
	std::vector <pho::api::PhoXiDeviceInformation> DeviceList;
	pho::api::PPhoXi Scanner;
	pho::api::PhoXiRawAccessHandler RawAccessHandler(Scanner);
	pho::PDataManager InitialSettings;
	pho::PDataManager InitialStructure;

	if (!FromFiles) {
		Factory.StartConsoleOutput("Admin-On");

		std::cout << "Connect first attached (1) or by IP (0)" << std::endl;
		bool FirstAttached;
		std::cin >> FirstAttached;
		if (FirstAttached) {
			DeviceList = Factory.GetDeviceList();
			if (DeviceList.empty()) return;

			pho::api::PhoXiDeviceInformation ChoosenDeviceInfo = DeviceList.back();
			for (std::size_t i = 0; i < DeviceList.size(); i++) {
				if (DeviceList[i].HWIdentification == ChoosenDeviceID) {
					ChoosenDeviceInfo = DeviceList[i];
				}
			}

			/*Scanner = Factory.Create(DeviceList.back());
			Scanner->Connect();*/
			Scanner = Factory.CreateAndConnectFirstAttached();
		}
		else {
			pho::api::PhoXiFactory Factory;
			//Check if the PhoXi Control Software is running
			if (!Factory.isPhoXiControlRunning()) return;
			std::cout << "Write ID: " << std::endl;
			std::string ID;
			std::cin >> ID;
			std::cout << "Write IP Adress: " << std::endl;
			std::string IP;
			std::cin >> IP;
			Scanner = Factory.CreateAndConnect(ID, pho::api::PhoXiDeviceType::PhoXiScanner, IP);
		}
		if (!Scanner->isConnected()) return;

		if (Scanner->TriggerMode != pho::api::PhoXiTriggerMode::Software) {
			if (Scanner->isAcquiring()) {
				Scanner->StopAcquisition();
				Scanner->TriggerMode = pho::api::PhoXiTriggerMode::Software;
			}
		}
		Scanner->ClearBuffer();
		Scanner->StartAcquisition();

		RawAccessHandler = pho::api::PhoXiRawAccessHandler(Scanner);

		InitialSettings = RawAccessHandler.GetSettings();
		InitialStructure = RawAccessHandler.GetStructure();
		if (!InitialSettings || !InitialStructure) return;
		//double DefaultSweepUpTime = 20.0;// InitialSettings->GetLeaf(SweepUpTimePath);
		//double DefaultSweepDownTime = 6.0;// InitialSettings->GetLeaf(SweepDownTimePath);

		DefaultSweepUpTime = InitialSettings->GetLeaf(SweepUpTimePath);
		double DefaultSweepDownTime = InitialSettings->GetLeaf(SweepDownTimePath);

		InitialStructure->GetLeaf(IndexMapPath).SetSerializeInSelectiveMode(true, true);

		{
			pho::DataManager ToSet;
			ToSet[UseCalibratedIndexOffsetsPath] = pho::AccessHandlerBox<bool>(false);
			RawAccessHandler.SetSettings(ToSet);
		}

		RawAccessHandler.SetStructure(*InitialStructure);

		//cv::Mat DefaultIndexMap = pho::Phoio::imRead("Output/index.Mat20.tif");////CaptureAndGetIndexMap(Scanner);
	}
	else {
		DefaultSweepUpTime = SupportedSweepUpTimes[DefaultSweepUpTimeIndex];
	}
	cv::Mat DefaultIndexMap;
	if (!FromFiles) {
		DefaultIndexMap = CaptureAndGetIndexMap(Scanner);
	}
	else {
		DefaultIndexMap = pho::Phoio::imRead(ImageFilePaths[DefaultSweepUpTimeIndex]);
	}
	if (DefaultIndexMap.empty()) return;

	std::vector<double> Offsets;

	pho::DataManager SupportedSweepUpTimesDM;

	pho::Phoio::imWrite("Output/OffsetCalibrationIndexMap(" + std::to_string(DefaultSweepUpTime) + ").tif",
		DefaultIndexMap);
	pho::Phoio::imWrite("Output/DefaultCalibrationIndexMap(" + std::to_string(DefaultSweepUpTime) + ").tif",
		DefaultIndexMap);

	for (std::size_t i = 0; i < SupportedSweepUpTimes.size(); i++) {
		if (SupportedSweepUpTimes[i] == DefaultSweepUpTime) {
			SupportedSweepUpTimesDM["Data/" + std::to_string(SupportedSweepUpTimes[i]) + ".double"] = 0.0;
			Offsets.push_back(0.0);
			continue;
		}

		if (!FromFiles) {
			pho::DataManager ToSet;
			ToSet[SweepUpTimePath] = pho::AccessHandlerBox<double>(SupportedSweepUpTimes[i]);
			RawAccessHandler.SetSettings(ToSet);
			RawAccessHandler.SetStructure(*InitialStructure);
		}

		//cv::Mat CurrentIndexMap = pho::Phoio::imRead("Output/index.Mat40.tif"); //CaptureAndGetIndexMap(Scanner);

		cv::Mat CurrentIndexMap;
		if (!FromFiles) {
			CurrentIndexMap = CaptureAndGetIndexMap(Scanner);
		}
		else {
			CurrentIndexMap = pho::Phoio::imRead(ImageFilePaths[i]);
		}
		if (CurrentIndexMap.empty()) return;

		pho::Phoio::imWrite("Output/OffsetCalibrationIndexMap(" + std::to_string(SupportedSweepUpTimes[i]) + ").tif",
			CurrentIndexMap);

		cv::Mat Difference = DefaultIndexMap - CurrentIndexMap;
		cv::Mat DifferenceMedian;
		cv::medianBlur(Difference, DifferenceMedian, 5);
		std::vector<float> AllDifferences;
		std::vector<std::pair<double, double>> DifferenceClusters(513, std::pair<double, double>(0.0, 0.0));
		std::vector < std::vector < double>> DifferenceIndexClusters(513);
		std::vector<double> DifferencePerIndex(513, -1000.0);
		for (int y = 0; y < Difference.rows; y++) {
			for (int x = 0; x < Difference.cols; x++) {
				float DefaultIndexMapValue = DefaultIndexMap.at<float>(y, x);
				float DifferenceMedianValue = DifferenceMedian.at<float>(y, x);
				if (DefaultIndexMapValue < IndexMin || DefaultIndexMapValue > IndexMax) continue;
				if (DifferenceMedianValue < IndexMin || DifferenceMedianValue > IndexMax) continue;
				float DifferencePixel = Difference.at<float>(y, x);
				float DifferenceMedianPixel = DifferenceMedian.at<float>(y, x);
				if (abs(DifferencePixel - DifferenceMedianPixel) > Tolerance) continue;
				DifferenceClusters[(int)DefaultIndexMapValue].first += DifferencePixel;
				DifferenceClusters[(int)DefaultIndexMapValue].second += 1.0;
				DifferenceIndexClusters[(int)DefaultIndexMapValue].push_back(DifferencePixel);
				AllDifferences.push_back(DifferencePixel);
			}
		}
		for (std::size_t Index = 0; Index < DifferenceClusters.size(); ++Index) {

			if (DifferenceClusters[Index].second > 0.0) DifferenceClusters[Index].first /= DifferenceClusters[Index].second;
			std::sort(DifferenceIndexClusters[Index].begin(), DifferenceIndexClusters[Index].end());
			double Diff = 0.0;
			if (FindClusterDiff(DifferenceIndexClusters[Index], Diff)) {
				DifferencePerIndex[Index] = Diff;
			}
		}
		std::sort(AllDifferences.begin(), AllDifferences.end());

		if (AllDifferences.size() < 10) return;
		std::size_t ClusterBegin = 0;
		std::size_t ClusterEnd = 0;
		double ClusterSum = 0.0;
		double ClusterMedian;
		double ClusterMean;
		std::size_t ClusterSize;

		std::size_t MaxClusterSize = 0;
		std::size_t MaxClusterBegin, MaxClusterEnd;
		double MaxClusterMean, MaxClusterMedian;

		std::vector<double> ClusterMeans;
		std::vector <std::size_t> ClusterSizes;

		while (ClusterEnd < AllDifferences.size()
			&& AllDifferences[ClusterEnd] - AllDifferences[ClusterBegin] <= ClusterWidth) {
			ClusterSum += AllDifferences[ClusterEnd];
			ClusterEnd++;
		}
		ClusterMedian = AllDifferences[(ClusterBegin + ClusterEnd) / 2];
		ClusterMean = ClusterSum / double(ClusterEnd - ClusterBegin);
		ClusterSize = ClusterEnd - ClusterBegin;
		MaxClusterSize = ClusterSize;
		MaxClusterBegin = ClusterBegin;
		MaxClusterEnd = ClusterEnd;
		ClusterMeans.push_back(ClusterMean);
		ClusterSizes.push_back(ClusterSize);

		for (ClusterEnd++; ClusterEnd <= AllDifferences.size(); ClusterEnd++) {
			ClusterSum += AllDifferences[ClusterEnd - 1];
			while (AllDifferences[ClusterEnd - 1] - AllDifferences[ClusterBegin] > ClusterWidth) {
				ClusterSum -= AllDifferences[ClusterBegin];
				ClusterBegin++;
			}
			ClusterMedian = AllDifferences[(ClusterBegin + ClusterEnd) / 2];
			ClusterMean = ClusterSum / double(ClusterEnd - ClusterBegin);
			ClusterSize = ClusterEnd - ClusterBegin;
			ClusterMeans.push_back(ClusterMean);
			ClusterSizes.push_back(ClusterSize);
			if (ClusterEnd - ClusterBegin > MaxClusterSize) {
				MaxClusterSize = ClusterSize;
				MaxClusterBegin = ClusterBegin;
				MaxClusterEnd = ClusterEnd;
				MaxClusterMedian = ClusterMedian;
				MaxClusterMean = ClusterMean;
			}
		}

		SupportedSweepUpTimesDM["Data/" + std::to_string(SupportedSweepUpTimes[i]) + ".double"] = MaxClusterMean;
		Offsets.push_back(MaxClusterMean);

		cv::Size DebugFrameSize(1000, 500);
		cv::Size2d Range((double)ClusterWidth * 2.0, MaxClusterSize);
		cv::Mat DebugImage = cv::Mat::zeros(DebugFrameSize, CV_8U);

		double RangeRadius = Range.width / 2.0;
		double RangeStart = MaxClusterMean - RangeRadius;
		for (std::size_t k = 0; k < ClusterMeans.size(); k++) {
			if (abs(ClusterMeans[k] - MaxClusterMean) < RangeRadius) {
				cv::Point2d Position;
				Position.x = ((ClusterMeans[k] - RangeStart) / Range.width) * (double)DebugFrameSize.width;
				Position.y = DebugFrameSize.height
					- ((double)ClusterSizes[k] / (double)MaxClusterSize) * (double)DebugFrameSize.height;
				cv::Point PositionInt = (Position);
				if (PositionInt.x >= 0 && PositionInt.x < DebugImage.cols && PositionInt.y >= 0
					&& PositionInt.y < DebugImage.rows) {
					DebugImage.at < unsigned
						char >(PositionInt.y, PositionInt.x) = 255;
				}
			}
		}
		//cv::imshow("pekne", DebugImage);
		cv::imwrite("Output/OffsetCalibration(" + std::to_string(i) + ")-[" + std::to_string(MaxClusterMean) + " , "
			+ std::to_string(MaxClusterMedian) + " , " + std::to_string(MaxClusterSize) + "].png",
			DebugImage);
		//cv::waitKey(0);
	}

	if (!FromFiles) {
		RawAccessHandler.SetSettings(*InitialSettings);

		SupportedSweepUpTimesDM["DefaultSweepUpTime.double"] = DefaultSweepUpTime;
	}

	cv::FileStorage fs("Output/IndexMapOffsets.xml", cv::FileStorage::WRITE);
	if (fs.isOpened()) {
		fs << "OffsetCalibration" << SupportedSweepUpTimesDM;
		fs.release();
	}
	else {
		return;
	}
}
#if 0
void CapturingExample() {
	pho::api::PhoXiFactory Factory;
	//Check if the PhoXi Control Software is running
	if (!Factory.isPhoXiControlRunning()) return;
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
		return;
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
		if (PhoXiDevice->isAcquiring()) {
			for (int i = 0; i < 5; i++) {
				std::cout << "Triggering the " << i << "-th frame" << std::endl;
				std::string CustomMessage = "Hello World - Software Trigger Frame Number: " + std::to_string(i);
				int FrameID = PhoXiDevice->TriggerFrame(true, false, CustomMessage);
				if (FrameID < 0) {
					//If negative number is returned trigger was unsuccessful
					std::cout << "Trigger was unsuccessful!" << std::endl;
					continue;
				}
				else {
					std::cout << "Frame was triggered, Frame Id: " << FrameID << std::endl;
				}
				std::cout << "Waiting for frame " << i << std::endl;
				pho::api::PFrame Frame = PhoXiDevice->GetSpecificFrame(FrameID, pho::api::PhoXiTimeout::Infinity);
				if (Frame) {
					std::cout << "Frame retrieved" << std::endl;
					std::cout << "  Frame params: " << std::endl;
					std::cout << "    Frame Index: " << Frame->Info.FrameIndex << std::endl;
					std::cout << "    Frame Timestamp: " << Frame->Info.FrameTimestamp << std::endl;
					std::cout << "    Frame Duration: " << Frame->Info.FrameDuration << std::endl;
					std::cout << "    Frame Resolution: " << Frame->GetResolution().Width << " x "
						<< Frame->GetResolution().Height << std::endl;
					std::cout << "    Sensor Position: " << Frame->Info.SensorPosition.x << "; "
						<< Frame->Info.SensorPosition.y << "; " << Frame->Info.SensorPosition.z << std::endl;
					std::cout << "    Frame Custom Message: " << Frame->CustomMessage << std::endl;
					if (!Frame->Empty()) {
						std::cout << "  Frame data: " << std::endl;
						if (!Frame->PointCloud.Empty()) {
							std::cout << "    PointCloud: " << Frame->PointCloud.Size.Width << " x "
								<< Frame->PointCloud.Size.Height << " Type: "
								<< Frame->PointCloud.GetElementName() << std::endl;
						}
						if (!Frame->NormalMap.Empty()) {
							std::cout << "    NormalMap: " << Frame->NormalMap.Size.Width << " x "
								<< Frame->NormalMap.Size.Height << " Type: " << Frame->NormalMap.GetElementName()
								<< std::endl;
						}
						if (!Frame->DepthMap.Empty()) {
							std::cout << "    DepthMap: " << Frame->DepthMap.Size.Width << " x "
								<< Frame->DepthMap.Size.Height << " Type: " << Frame->DepthMap.GetElementName()
								<< std::endl;
						}
						if (!Frame->ConfidenceMap.Empty()) {
							std::cout << "    ConfidenceMap: " << Frame->ConfidenceMap.Size.Width << " x "
								<< Frame->ConfidenceMap.Size.Height << " Type: "
								<< Frame->ConfidenceMap.GetElementName() << std::endl;
						}
						if (!Frame->Texture.Empty()) {
							std::cout << "    Texture: " << Frame->Texture.Size.Width << " x "
								<< Frame->Texture.Size.Height << " Type: " << Frame->Texture.GetElementName()
								<< std::endl;
						}
					}
					else {
						std::cout << "Frame is empty.";
					}
				}
				else {
					std::cout << "Failed to retrieve the frame!";
				}
			}
		}

		PhoXiDevice->StopAcquisition();
		std::cout << "Starting Freerun mode" << std::endl;
		PhoXiDevice->TriggerMode = pho::api::PhoXiTriggerMode::Freerun;
		PhoXiDevice->StartAcquisition();

		if (PhoXiDevice->isAcquiring()) {
			for (int i = 0; i < 5; i++) {
				std::cout << "Waiting for frame " << i << std::endl;
				pho::api::PFrame Frame = PhoXiDevice->GetFrame(pho::api::PhoXiTimeout::Infinity);
				if (Frame) {
					std::cout << "Frame retrieved" << std::endl;
					std::cout << "  Frame params: " << std::endl;
					std::cout << "    Frame Index: " << Frame->Info.FrameIndex << std::endl;
					std::cout << "    Frame Timestamp: " << Frame->Info.FrameTimestamp << std::endl;
					std::cout << "    Frame Duration: " << Frame->Info.FrameDuration << std::endl;
					std::cout << "    Frame Resolution: " << Frame->GetResolution().Width << " x "
						<< Frame->GetResolution().Height << std::endl;
					std::cout << "    Sensor Position: " << Frame->Info.SensorPosition.x << "; "
						<< Frame->Info.SensorPosition.y << "; " << Frame->Info.SensorPosition.z << std::endl;
					if (!Frame->Empty()) {
						std::cout << "  Frame data: " << std::endl;
						if (!Frame->PointCloud.Empty()) {
							std::cout << "    PointCloud: " << Frame->PointCloud.Size.Width << " x "
								<< Frame->PointCloud.Size.Height << " Type: "
								<< Frame->PointCloud.GetElementName() << std::endl;
						}
						if (!Frame->NormalMap.Empty()) {
							std::cout << "    NormalMap: " << Frame->NormalMap.Size.Width << " x "
								<< Frame->NormalMap.Size.Height << " Type: " << Frame->NormalMap.GetElementName()
								<< std::endl;
						}
						if (!Frame->DepthMap.Empty()) {
							std::cout << "    DepthMap: " << Frame->DepthMap.Size.Width << " x "
								<< Frame->DepthMap.Size.Height << " Type: " << Frame->DepthMap.GetElementName()
								<< std::endl;
						}
						if (!Frame->ConfidenceMap.Empty()) {
							std::cout << "    ConfidenceMap: " << Frame->ConfidenceMap.Size.Width << " x "
								<< Frame->ConfidenceMap.Size.Height << " Type: "
								<< Frame->ConfidenceMap.GetElementName() << std::endl;
						}
						if (!Frame->Texture.Empty()) {
							std::cout << "    Texture: " << Frame->Texture.Size.Width << " x "
								<< Frame->Texture.Size.Height << " Type: " << Frame->Texture.GetElementName()
								<< std::endl;
						}
					}
					else {
						std::cout << "Frame is empty.";
					}
				}
				else {
					std::cout << "Failed to retrieve the frame!";
				}
			}
		}
		PhoXiDevice->StopAcquisition();
	}
	PhoXiDevice->Disconnect();

	return;

#if 0
	pho::api::PhoXiFactory Factory;
	Factory.StartConsoleOutput("Admin-On");
	while (!Factory.isPhoXiControlRunning()) {
		boost::this_thread::sleep_for(boost::chrono::milliseconds(1000));
	}
	for (int k = 0; k < 1; k++) {
		std::vector<pho::api::PhoXiDeviceInformation> DeviceList = Factory.GetDeviceList();
		if (!DeviceList.empty()) {
			//            std::string camera_hw_identification = "noConnectedCamera-03";
			//            int index = 0;
			//            for (int i = 0; i < DeviceList.size(); i++) {
			//                if (DeviceList[i].HWIdentification == camera_hw_identification) {
			//                    index = i;
			//                }
			//            }
			//            pho::api::PPhoXi EvaluationScanner = Factory.Create(DeviceList[index]);
			//            pho::api::PPhoXi EvaluationScanner = Factory.Create(DeviceList.back());
			pho::api::PPhoXi EvaluationScanner = Factory.CreateAndConnectFirstAttached();
			if (!EvaluationScanner) {
				k--;
				boost::this_thread::sleep_for(boost::chrono::milliseconds(1000));
				continue;
				//return;
			}
			EvaluationScanner->Connect();
			if (EvaluationScanner->isAcquiring()) {
				EvaluationScanner->StopAcquisition();
			}

			if (EvaluationScanner->isConnected()) {
				/*std::vector<pho::api::PhoXiCapturingMode> SupportedModes = EvaluationScanner->SupportedCapturingModes;
				EvaluationScanner->CapturingMode = SupportedModes[1];*/

				pho::api::PhoXiSize Resolution = EvaluationScanner->Resolution;
				int Width = EvaluationScanner->Resolution->Width;
				int Heght = EvaluationScanner->Resolution->Height;
				/*while (true) {
				if (EvaluationScanner->GetFrame(1000)) {
				std::cout << "NewFrameArrived" << std::endl;
				}
				}*/


				EvaluationScanner->TriggerMode = pho::api::PhoXiTriggerMode::Software;
				pho::api::PhoXiRawAccessHandler RawAccessHandler(EvaluationScanner);
				pho::PDataManager DM = RawAccessHandler.GetSettings();
				/*if (DM) {
				std::cout << DM->GetStringSerialization() << std::endl;
				}*/

				while (EvaluationScanner->GetFrame(0)) {
					std::cout << "Cleaning Buffer" << std::endl;
				}
				EvaluationScanner->StartAcquisition();
				if (EvaluationScanner->isAcquiring()) {
					for (int i = 0; i < 10; i++) {
						int FrameID = EvaluationScanner->TriggerImage();
						if (FrameID >= 0) {
							pho::api::PFrame MyFrame = EvaluationScanner->GetFrame(pho::api::PhoXiTimeout::Infinity);
							if (MyFrame) {
								pho::PDataManager DM = RawAccessHandler.GetLastOutput();
								std::cout << i << std::endl;
								if (!MyFrame->PointCloud.Empty()) std::cout << "PointCloud: " << MyFrame->PointCloud.Size.Width << " x " << MyFrame->PointCloud.Size.Height << " Type: " << MyFrame->PointCloud.GetElementName() << std::endl;
								if (!MyFrame->NormalMap.Empty()) std::cout << "NormalMap: " << MyFrame->NormalMap.Size.Width << " x " << MyFrame->NormalMap.Size.Height << " Type: " << MyFrame->NormalMap.GetElementName() << std::endl;
								if (!MyFrame->DepthMap.Empty()) std::cout << "DepthMap: " << MyFrame->DepthMap.Size.Width << " x " << MyFrame->DepthMap.Size.Height << " Type: " << MyFrame->DepthMap.GetElementName() << std::endl;
								if (!MyFrame->Texture.Empty()) {
									std::cout << "Texture: " << MyFrame->Texture.Size.Width << " x " << MyFrame->Texture.Size.Height << " Type: " << MyFrame->Texture.GetElementName() << std::endl;
									cv::Mat Texture;
									MyFrame->Texture.ConvertTo(Texture);
									cv::Mat Texture16;
									Texture.convertTo(Texture16, CV_16U);
									cv::imwrite("TextTexture" + std::to_string(i) + ".png", Texture16);
								}
								if (!MyFrame->ConfidenceMap.Empty()) std::cout << "ConfidenceMap: " << MyFrame->ConfidenceMap.Size.Width << " x " << MyFrame->ConfidenceMap.Size.Height << " Type: " << MyFrame->ConfidenceMap.GetElementName() << std::endl;
								MyFrame->SaveAsPly("Test Software" + std::to_string(k) + " , " + std::to_string(i) + ".ply");

								/*pcl::PointCloud<pcl::PointXYZRGB> MyPCLCloud;
								MyFrame->ConvertTo(MyPCLCloud);
								pcl::PCLPointCloud2 MyPCLCloud2;
								MyFrame->ConvertTo(MyPCLCloud2);
								pcl::PLYWriter Writer;*/

								//Writer.writeBinary("Test Software PCL" + std::to_string(k) + " , " + std::to_string(i) + ".ply", MyPCLCloud2);
							}
						}
					}
				}

				//EvaluationScanner->StopAcquisition();
				//EvaluationScanner->TriggerMode = pho::api::PhoXiTriggerMode::Freerun;

				//EvaluationScanner->StartAcquisition();
				//if (EvaluationScanner->isAcquiring()) {
				//    for (int i = 0; i < 1; i++) {
				//        pho::api::PFrame MyFrame = EvaluationScanner->GetFrame(pho::api::PhoXiTimeout::Infinity);
				//        if (MyFrame) {
				//            std::cout << i << std::endl;
				//            if (!MyFrame->PointCloud.Empty()) std::cout << "PointCloud: " << MyFrame->PointCloud.Size.Width << " x " << MyFrame->PointCloud.Size.Height << " Type: " << MyFrame->PointCloud.GetElementName() << std::endl;
				//            if (!MyFrame->DepthMap.Empty()) std::cout << "DepthMap: " << MyFrame->DepthMap.Size.Width << " x " << MyFrame->DepthMap.Size.Height << " Type: " << MyFrame->DepthMap.GetElementName() << std::endl;
				//            if (!MyFrame->Texture.Empty()) std::cout << "Texture: " << MyFrame->Texture.Size.Width << " x " << MyFrame->Texture.Size.Height << " Type: " << MyFrame->Texture.GetElementName() << std::endl;
				//            if (!MyFrame->ConfidenceMap.Empty()) std::cout << "ConfidenceMap: " << MyFrame->ConfidenceMap.Size.Width << " x " << MyFrame->ConfidenceMap.Size.Height << " Type: " << MyFrame->ConfidenceMap.GetElementName() << std::endl;
				//            //MyFrame->SaveAsPly("Test Freerun" + std::to_string(k) + " , " + std::to_string(i) + ".ply");
				//        }
				//    }
				//}
				//EvaluationScanner->StopAcquisition();


			}
			EvaluationScanner->Disconnect();
		}
	}
#endif
}

void SoftTriggerWaitExample() {
	boost::this_thread::sleep_for(boost::chrono::milliseconds(5000));
	pho::api::PhoXiFactory Factory;
	//Check if the PhoXi Control Software is running
	if (!Factory.isPhoXiControlRunning()) return;

	pho::api::PPhoXi Gorazd = Factory.CreateAndConnect("20160324004");
	pho::api::PPhoXi Rastislav = Factory.CreateAndConnect("1607003");

	if (Gorazd && Rastislav) {
		Gorazd->StopAcquisition();
		Rastislav->StopAcquisition();
		Gorazd->TriggerMode = pho::api::PhoXiTriggerMode::Software;
		Rastislav->TriggerMode = pho::api::PhoXiTriggerMode::Software;
		Gorazd->StartAcquisition();
		Rastislav->StartAcquisition();
		pho::api::PFrame GorazdFrame;
		pho::api::PFrame RastislavFrame;
		for (int i = 0; i < 100; i++) {
			int GorazdFrameID = Gorazd->TriggerFrame(true, true);
			int RastislavFrameID = Rastislav->TriggerFrame(true, true);
			GorazdFrame = Gorazd->GetSpecificFrame(GorazdFrameID);
			RastislavFrame = Rastislav->GetSpecificFrame(RastislavFrameID);

		}
		if (GorazdFrame && RastislavFrame) {
			GorazdFrame->SaveAsPly("Gorazd.ply");
			RastislavFrame->SaveAsPly("Rastislav.ply");
		}
	}
}

void CameraCalibrateModeExample() {
	pho::api::PhoXiFactory Factory;
	//Check if the PhoXi Control Software is running
	if (!Factory.isPhoXiControlRunning()) return;
	std::cout << "Write ID: " << std::endl;
	std::string ID;
	std::cin >> ID;

	pho::api::PPhoXi PhoXiInstance = Factory.Create(pho::api::PhoXiDeviceType::PhoXiScanner);
	PhoXiInstance->HardwareIdentification = ID;
	pho::api::PhoXiRawAccessHandler RawHandler(PhoXiInstance);
	RawHandler.SetMode("CameraStream");
	PhoXiInstance->Connect();
	if (PhoXiInstance->isConnected()) {
		PhoXiInstance->TriggerMode = pho::api::PhoXiTriggerMode::Software;
		PhoXiInstance->StartAcquisition();

		std::vector<pho::PCamera> Cameras = RawHandler.GetPCameras();

		for (int i = 0; i < 10; i++) {
			std::cout << i << " Before Trigger" << std::endl;
			int TriggerID = PhoXiInstance->TriggerFrame();
			std::cout << i << " Before Get" << " " << TriggerID << std::endl;
			PhoXiInstance->GetSpecificFrame(TriggerID);
			std::cout << i << " Before AllOutput" << std::endl;
			RawHandler.GetLastOutput();
			std::cout << i << " After" << std::endl;
			std::cout << i << std::endl;
		}
	}
}
void IPConnectExample() {
	pho::api::PhoXiFactory Factory;
	//Check if the PhoXi Control Software is running
	if (!Factory.isPhoXiControlRunning()) return;
	std::cout << "Write ID: " << std::endl;
	std::string ID;
	std::cin >> ID;
	std::cout << "Write IP Adress: " << std::endl;
	std::string IP;
	std::cin >> IP;
	pho::api::PPhoXi PhoXiInstance = Factory.CreateAndConnect(ID, pho::api::PhoXiDeviceType::PhoXiScanner, IP);
	if (PhoXiInstance) PhoXiInstance->Disconnect(false, false);
}

class NormalHistogram {
private:
	cv::Mat Histogram;
	cv::Mat Acumulator;
	int HorizontalResolution, VerticalResolution;
	float AngleConversionHorizontal, AngleConversionVertical;
public:
	NormalHistogram(int _HorizontalResolution, int _VerticalResolution) : HorizontalResolution(_HorizontalResolution), VerticalResolution(_VerticalResolution) {
		Histogram = cv::Mat::zeros(cv::Size(HorizontalResolution, VerticalResolution), CV_32SC1);
		Acumulator = cv::Mat::zeros(cv::Size(HorizontalResolution, VerticalResolution), CV_32SC1);
		AngleConversionHorizontal = (float)(HorizontalResolution - 1) / 360.0f;
		AngleConversionVertical = (float)(VerticalResolution - 1) / 180.0;
	}
	inline bool AddNormal(const cv::Point3f& Normal) {
		static const cv::Point3f ZeroPoint = cv::Point3f(0.0f, 0.0f, 0.0f);
		if (Normal == ZeroPoint) return false;
		float HorizontalAngle = cv::fastAtan2(Normal.y, Normal.x);
		float Length = std::sqrt(Normal.x * Normal.x + Normal.y * Normal.y);
		float VerticalAngle = cv::fastAtan2(Length, Normal.z);
		Histogram.at<int>(int(VerticalAngle * AngleConversionVertical) % VerticalResolution, int(HorizontalAngle * AngleConversionHorizontal) % HorizontalResolution)++;
		return true;
	}
	void SaveHistogram() {
		cv::Mat Histogram32f;
		Histogram.convertTo(Histogram32f, CV_32FC1);
		pho::imWrite("Histogram.tif", Histogram32f);
	}
	void CreateAcumulator() {

	}
};

void ConnectedComponentsTest() {

	pho::DataManager Test;

	pho::AccessHandlerBox<double>& Test2 = Test.AllocateIfEmpty("Niekto/Niekde/Isto.AHBox<double>", pho::AccessHandlerBox < double>(5.0)).Ref<pho::AccessHandlerBox<double>>();

	Test2;



	pho::api::PhoXiFactory Factory;
	pho::api::PPhoXi ScannerInstance = Factory.CreateAndConnectFirstAttached();
	if (ScannerInstance) {
		ScannerInstance->TriggerMode = pho::api::PhoXiTriggerMode::Software;

		ScannerInstance->StartAcquisition();
		pho::api::PFrame Frame = ScannerInstance->GetSpecificFrame(ScannerInstance->TriggerFrame());
		if (Frame && !Frame->PointCloud.Empty() && !Frame->NormalMap.Empty()) {
			cv::Mat NormalMap, PointCloud;
			cv::Point3f ZeroPoint = cv::Point3f(0.0f, 0.0f, 0.0f);
			Frame->NormalMap.ConvertTo(NormalMap);
			Frame->PointCloud.ConvertTo(PointCloud);
			NormalHistogram Histogram(256, 128);
			for (int y = 0; y < NormalMap.rows; y++) {
				cv::Point3f* HistogramRow = NormalMap.ptr<cv::Point3f>(y);
				cv::Point3f* PointCloudRow = PointCloud.ptr<cv::Point3f>(y);
				for (int x = 0; x < NormalMap.cols; x++) {
					if (*PointCloudRow != ZeroPoint) {
						Histogram.AddNormal(*HistogramRow);
					}
					++HistogramRow;
					++PointCloudRow;
				}

			}
			Histogram.SaveHistogram();
		}
	}
}


void AnalyzeSTDFromDataSet(const std::vector<cv::Mat>& Input, float MinZ, float MaxZ, int Chunks) {
	std::vector<std::vector<float>> SingleBandSamples(Chunks);
	for (std::size_t i = 0; i < Input.size(); i++) {
		cv::Mat PointCloud = Input[i];
		cv::Mat PointCloudSmooth;

		int SmoothRadius = 5;
		double MaxZDiff;

		cv::Point3f ZeroPoint = cv::Point3f(0.0f, 0.0f, 0.0f);
		for (int y = SmoothRadius; y < PointCloud.rows - SmoothRadius; y++) {
			for (int x = SmoothRadius; x < PointCloud.cols - SmoothRadius; x++) {
				std::vector<cv::Point3f> Points;
				for (int yy = y - SmoothRadius; yy <= y + SmoothRadius; yy++) {
					for (int xx = x - SmoothRadius; xx <= x + SmoothRadius; xx++) {
						cv::Point3f& ActualPoint = PointCloud.at<cv::Point3f>(yy, xx);
						if (ActualPoint != ZeroPoint) Points.push_back(ActualPoint);

					}
				}
			}
		}
	}
}
void StandartDeviationTest(float MinZ, float MaxZ, int Chunks) {
	pho::api::PhoXiFactory Factory;
	pho::api::PPhoXi Device = Factory.CreateAndConnectFirstAttached();
	std::vector<cv::Mat> AllSamples;
	if (!Device) return;
	while (true) {
		pho::api::PFrame Frame = Device->GetFrame();
		if (!Frame || Frame->Empty() || Frame->DepthMap.Empty()) continue;
		cv::Mat DepthMap;
		Frame->DepthMap.ConvertTo(DepthMap);
		AllSamples.push_back(DepthMap);
		AnalyzeSTDFromDataSet(AllSamples, MinZ, MaxZ, Chunks);
	}
	return;
}

#include <fstream>
//#include <boost/archive/basic_binary_iarchive.hpp>
//#include <boost/archive/basic_binary_oarchive.hpp>
//#include <boost/archive/binary_oarchive.hpp>
//#include <boost/archive/binary_iarchive.hpp>
//#include <boost/archive/text_oarchive.hpp>
//#include <boost/serialization/version.hpp>
//#include "InterprocessTools/InterprocessCommandHelperFunctions.h"

#endif

#pragma pack(push, 1)
#pragma pack(1)
#pragma pack(pop)

namespace cv {
	template<class T, class K>
	void write(cv::FileStorage &fs, const cv::String &name, const std::pair<T, K> &r) {
		fs << "{";
		fs << "first" << r.first;
		fs << "second" << r.second;
		fs << "}";
	}

	template<class T, class K>
	void read(const cv::FileNode &node, std::pair<T, K> &x, const std::pair<T, K> &default_value = std::pair<T, K>()) {
		if (node.empty()) {
			x = default_value;
		}
		else {
			node["first"] >> x.first;
			node["second"] >> x.second;
			//x.read(node);
		}
	}

	void write(cv::FileStorage &fs, const cv::String &name, const pho::Transformation3D64 &r) {
		cv::Mat Transformation = r;
		fs << Transformation;
	}

	void read(const cv::FileNode &node, pho::Transformation3D64 &x, const pho::Transformation3D64 &default_value = pho::Transformation3D64()) {
		if (node.empty()) {
			x = default_value;
		}
		else {
			cv::Mat Transformation;
			node >> Transformation;
			x = pho::Transformation3D64(Transformation);
			//x.read(node);
		}
	}
}


#define ITERATIVE_TRANSLATION_VECTOR_MULTIPLIER -1.0

namespace pho {
	namespace CameraCalibration {

		template <int DistortionCoefficientsCount, int Order, typename T>
		class CameraIntrinsicParameters {
		public:
			T Data[2 + 2 + DistortionCoefficientsCount * (Order + 1)];
			T* GetData() {
				return Data;
			}
			T* GetCameraFOV() {
				return Data;
			}
			T* GetCameraPrincipalPoint() {
				return &Data[2];
			}
			void SetCameraDistortion(T* DistortionCoeffs, int Index = 0) {
				T* DistortionCoeffsInternal = GetCameraDistortion(Index);
				for (int k = 0; k < DistortionCoefficientsCount; ++k) {
					DistortionCoeffsInternal[k] = DistortionCoeffs[k];
				}
			}

			T* GetCameraDistortion(int Index = 0) {
				return &Data[4 + Index * DistortionCoefficientsCount];
			}
			CameraIntrinsicParameters() {
				Data[0] = (T)1.0;
				Data[1] = (T)1.0;
				for (int i = 2; i < 2 + 2 + DistortionCoefficientsCount * (Order + 1); ++i) {
					Data[i] = (T)0.0;
				}
			}
			CameraIntrinsicParameters(const CameraIntrinsicParameters<DistortionCoefficientsCount, Order, T>& Other) {
				memcpy(Data, Other.Data, sizeof(T) * (2 + 2 + DistortionCoefficientsCount * (Order + 1)));
			}
			CameraIntrinsicParameters(const photoneoTools::CameraCalibration64& Calibration) {
				cv::Mat ActualCameraMatrix = Calibration.GetCameraMatrix();
				cv::Mat ActualDistortionCoefficientTemp = Calibration.GetDistortionCoefficients();
				cv::Mat ActualDistortionCoefficient(cv::Size(1, DistortionCoefficientsCount), CV_64FC1, cv::Scalar(0.0));
				if (ActualDistortionCoefficientTemp.rows > DistortionCoefficientsCount) {
					ActualDistortionCoefficientTemp(cv::Rect(0, 0, 1, DistortionCoefficientsCount)).copyTo(ActualDistortionCoefficient);
				}
				else {
					if (ActualDistortionCoefficientTemp.rows < DistortionCoefficientsCount) {
						pho_runtime_error("Incompatible number of Distortion Parameters");
					}
					ActualDistortionCoefficient = ActualDistortionCoefficientTemp;
				}
				//OutputCalibrations[CameraID].SetDistortionCoefficients(ActualDistortionCoefficient);

				GetCameraFOV()[0] = ActualCameraMatrix.at<double>(0, 0);
				GetCameraFOV()[1] = ActualCameraMatrix.at<double>(1, 1);
				GetCameraPrincipalPoint()[0] = ActualCameraMatrix.at<double>(0, 2);
				GetCameraPrincipalPoint()[1] = ActualCameraMatrix.at<double>(1, 2);

				T* DistortionCoeffs = GetCameraDistortion();
				for (int k = 0; k < DistortionCoefficientsCount; ++k) {
					DistortionCoeffs[k] = (T)ActualDistortionCoefficient.ptr<double>()[k];
				}
				for (int i = 1; i < Order + 1; ++i) {
					memcpy(GetCameraDistortion(i), GetCameraDistortion(0), sizeof(T) * DistortionCoefficientsCount);
				}
			}
		};

		template <int DistortionCoefficientsCount, int Order, typename T>
		class CameraExtendedIntrinsicParameters {
		public:
			T Data[(2 + 2 + DistortionCoefficientsCount) * (Order + 1)];
			T* GetData(int Index = 0) {
				return &Data[Index * (DistortionCoefficientsCount + 4)];
			}
			T* GetCameraFOV(int Index = 0) {
				return &Data[Index * (DistortionCoefficientsCount + 4)];
			}
			T* GetCameraPrincipalPoint(int Index = 0) {
				return &Data[Index * (DistortionCoefficientsCount + 4) + 2];
			}
			void SetCameraDistortion(T* DistortionCoeffs, int Index = 0) {
				T* DistortionCoeffsInternal = GetCameraDistortion(Index);
				for (int k = 0; k < DistortionCoefficientsCount; ++k) {
					DistortionCoeffsInternal[k] = DistortionCoeffs[k];
				}
			}

			void SetCameraIntrinsic(T* IntrinsicParameters, int Index = 0) {
				T* IntrinsicParametersInternal = GetData(Index);
				for (int k = 0; k < DistortionCoefficientsCount + 4; ++k) {
					IntrinsicParametersInternal[k] = IntrinsicParameters[k];
				}
			}

			T* GetCameraDistortion(int Index = 0) {
				return &Data[Index * (DistortionCoefficientsCount + 4) + 4];
			}
			CameraExtendedIntrinsicParameters() {
				for (int Index = 0; Index < Order + 1; ++Index) {
					GetCameraFOV(Index)[0] = (T)1.0;
					GetCameraFOV(Index)[1] = (T)1.0;
					GetCameraPrincipalPoint(Index)[0] = (T)0.0;
					GetCameraPrincipalPoint(Index)[1] = (T)0.0;
					for (int i = 0; i < DistortionCoefficientsCount; ++i) {
						GetCameraDistortion(Index)[i] = (T)0.0;
					}
				}
			}
			CameraExtendedIntrinsicParameters(const CameraExtendedIntrinsicParameters<DistortionCoefficientsCount, Order, T>& Other) {
				memcpy(Data, Other.Data, sizeof(T) * ((2 + 2 + DistortionCoefficientsCount) * (Order + 1)));
			}
			CameraExtendedIntrinsicParameters(const photoneoTools::CameraCalibration64& Calibration) {
				cv::Mat ActualCameraMatrix = Calibration.GetCameraMatrix();
				cv::Mat ActualDistortionCoefficientTemp = Calibration.GetDistortionCoefficients();
				cv::Mat ActualDistortionCoefficient(cv::Size(1, DistortionCoefficientsCount), CV_64FC1, cv::Scalar(0.0));
				if (ActualDistortionCoefficientTemp.rows > DistortionCoefficientsCount) {
					ActualDistortionCoefficientTemp(cv::Rect(0, 0, 1, DistortionCoefficientsCount)).copyTo(ActualDistortionCoefficient);
				}
				else {
					if (ActualDistortionCoefficientTemp.rows < DistortionCoefficientsCount) {
						pho_runtime_error("Incompatible number of Distortion Parameters");
					}
					ActualDistortionCoefficient = ActualDistortionCoefficientTemp;
				}
				//OutputCalibrations[CameraID].SetDistortionCoefficients(ActualDistortionCoefficient);

				GetCameraFOV()[0] = ActualCameraMatrix.at<double>(0, 0);
				GetCameraFOV()[1] = ActualCameraMatrix.at<double>(1, 1);
				GetCameraPrincipalPoint()[0] = ActualCameraMatrix.at<double>(0, 2);
				GetCameraPrincipalPoint()[1] = ActualCameraMatrix.at<double>(1, 2);

				T* DistortionCoeffs = GetCameraDistortion();
				for (int k = 0; k < DistortionCoefficientsCount; ++k) {
					DistortionCoeffs[k] = (T)ActualDistortionCoefficient.ptr<double>()[k];
				}

				for (int i = 1; i < Order + 1; ++i) {
					memcpy(GetData(i), GetData(0), sizeof(T) * (2 + 2 + DistortionCoefficientsCount));
				}
			}
		};

		template<typename T>
		class vectorBox : public std::vector<T> {
		public:
			vectorBox(const std::vector<T>& Other) : vector<T>(Other) {

			}
			/*template<typename K>
			vectorBox(const std::vector<std:vector<K>>& Other) : vector<T>((std::vector<vectorBox<K>)Other) {

			}*/
			vectorBox(const vectorBox<T>& Other) : vector<T>(Other) {

			}
			vectorBox() : vector<T>() {

			}
			virtual ~vectorBox() {

			}
			bool write(cv::FileStorage &fs) const {
				fs << "{";
				fs << "vectorData" << *(std::vector<T>*)(this);
				fs << "}";
				return true;
			}
			bool read(const cv::FileNode &node) {
				node["vectorData"] >> *(std::vector<T>*)(this);
				return true;
			}
		};


		template <int I, int N>
		struct MicroBersteinPolynomial {
			static const int64 InternalCombination = pho::Volumetric::Combination<N, I>::Value;
			template <typename T>
			static inline T Get(T u) {
				static T _InternalCombination = (T)InternalCombination;
				static T _NMinusI = (T)(N - I);
				static T _I = (T)I;
				return _InternalCombination * ceres::pow(((T)1.0 - u), _NMinusI) * ceres::pow(u, _I);
			}
		};
		template <int N, typename T>
		T MicroBersteinPolynomialUpTo10(int I, T u) {
			switch (I) {
			case 0: return MicroBersteinPolynomial<MIN(0, N), N> ::Get(u);
			case 1: return MicroBersteinPolynomial<MIN(1, N), N> ::Get(u);
			case 2: return MicroBersteinPolynomial<MIN(2, N), N> ::Get(u);
			case 3: return MicroBersteinPolynomial<MIN(3, N), N> ::Get(u);
			case 4: return MicroBersteinPolynomial<MIN(4, N), N> ::Get(u);
			case 5: return MicroBersteinPolynomial<MIN(5, N), N> ::Get(u);
			case 6: return MicroBersteinPolynomial<MIN(6, N), N> ::Get(u);
			case 7: return MicroBersteinPolynomial<MIN(7, N), N> ::Get(u);
			case 8: return MicroBersteinPolynomial<MIN(8, N), N> ::Get(u);
			case 9: return MicroBersteinPolynomial<MIN(9, N), N> ::Get(u);
			case 10: return MicroBersteinPolynomial<MIN(10, N), N> ::Get(u);
			}
			return (T)-1.0;
		}

		//Dim is order + 1
		template <int Dim>
		class MicroBezierWarper {
		public:
			const double XMin, YMin, ZMin, XDiffInv, YDiffInv, ZDiffInv;
			template <typename T>
			bool WarpPoint(const T* const InputPoint, T* OutputPoint, const T* const BezierPoints) const{
				T Result[3] = { (InputPoint[0] - XMin) * XDiffInv, (InputPoint[1] - YMin) * YDiffInv, (InputPoint[2] - YMin) * YDiffInv };
				OutputPoint[0] = (T)0.0;
				OutputPoint[1] = (T)0.0;
				OutputPoint[2] = (T)0.0;

				int Index = 0;
				for (int z = 0; z < Dim; z++) {
					for (int y = 0; y < Dim; y++) {
						for (int x = 0; x < Dim; x++) {
							T ActualFactor = MicroBersteinPolynomialUpTo10<Dim - 1>(z, Result[0]) * MicroBersteinPolynomialUpTo10<Dim - 1>(y, Result[1]) * MicroBersteinPolynomialUpTo10<Dim - 1>(x, Result[2]);
							OutputPoint[0] += ActualFactor * BezierPoints[Index++];
							OutputPoint[1] += ActualFactor * BezierPoints[Index++];
							OutputPoint[2] += ActualFactor * BezierPoints[Index++];
						}
					}
				}

				OutputPoint[0] += InputPoint[0];
				OutputPoint[1] += InputPoint[1];
				OutputPoint[2] += InputPoint[2];
				return true;
			}
			MicroBezierWarper(double XMin, double YMin, double ZMin, double XMax, double YMax, double ZMax) : XMin(XMin), YMin(YMin), ZMin(ZMin), XDiffInv(1.0 / (XMax - XMin)), YDiffInv(1.0 / (YMax - YMin)), ZDiffInv(1.0 / (ZMax - ZMin)) {

			}

			MicroBezierWarper(const MicroBezierWarper& Other) : XMin(Other.XMin), YMin(Other.YMin), ZMin(Other.ZMin), XDiffInv(Other.XDiffInv), YDiffInv(Other.YDiffInv), ZDiffInv(Other.ZDiffInv) {
			}

		};

		template <int Dim, int DistortionCoefficientsCount>
		class MicroBezierWarperDistortion {
		public:
			const double Min, DiffInv;
			template <typename T>
			bool GetDistiortion(const T* const DistortionCoefficients, const T& Depth, T* PerDepthDistortion) const {
				T DepthIndex = (Depth - Min) * DiffInv;
				for (int i = 0; i < DistortionCoefficientsCount; i++) {
					PerDepthDistortion[i] = (T)0.0;
				}

				int Index = 0;
				for (int z = 0; z < Dim; z++) {
					T ActualFactor = MicroBersteinPolynomialUpTo10<Dim - 1>(z, DepthIndex);
					for (int i = 0; i < DistortionCoefficientsCount; i++) {
						PerDepthDistortion[i] += ActualFactor * DistortionCoefficients[Index++];
					}
				}
				return true;
			}
			MicroBezierWarperDistortion(double Min, double Max) : Min(Min), DiffInv(1.0 / (Max - Min)) {

			}

			MicroBezierWarperDistortion(const MicroBezierWarperDistortion& Other) : Min(Other.Min), DiffInv(Other.DiffInv) {
			}

		};

		template <int Dim, int DistortionCoefficientsCount>
		class MicroBezierWarperIntrinsic {
		public:
			const double Min, DiffInv;
			template <typename T>
			bool GetIntrinsic(const T* const IntrinsicParams, const T& Depth, T* PerDepthIntrinsicParams, bool FixFOV = false, bool FixPrincipal = false) const {
				T DepthIndex = (Depth - Min) * DiffInv;
				for (int i = 0; i < DistortionCoefficientsCount + 4; i++) {
					PerDepthIntrinsicParams[i] = (T)0.0;
				}

				int Index = 0;
				for (int z = 0; z < Dim; z++) {
					T ActualFactor = MicroBersteinPolynomialUpTo10<Dim - 1>(z, DepthIndex);
					for (int i = 0; i < DistortionCoefficientsCount + 4; i++) {
						PerDepthIntrinsicParams[i] += ActualFactor * IntrinsicParams[Index++];
					}
				}
				if (FixFOV) {
					PerDepthIntrinsicParams[0] = IntrinsicParams[0];
					PerDepthIntrinsicParams[1] = IntrinsicParams[1];
				}
				if (FixPrincipal) {
					PerDepthIntrinsicParams[2] = IntrinsicParams[2];
					PerDepthIntrinsicParams[3] = IntrinsicParams[3];
				}
				return true;
			}
			MicroBezierWarperIntrinsic(double Min, double Max) : Min(Min), DiffInv(1.0 / (Max - Min)) {

			}

			MicroBezierWarperIntrinsic(const MicroBezierWarperIntrinsic& Other) : Min(Other.Min), DiffInv(Other.DiffInv) {
			}

		};


		template <int DistortionCoefficientsCount>
		class MeasuredCalibrationPointFunctor {
		public:

			template <typename T> void GetCameraCoordinate(
				const T* const PatternTranslationVector,
				const T* const PatternRotationQuaternion,
				const T* const CameraTranslationVector,
				const T* const CameraRotationQuaternion,
				T* Result) const {
				Result[0] = (T)PatternPoint[0];
				Result[1] = (T)PatternPoint[1];
				Result[2] = (T)PatternPoint[2];
				T Temp[3];

				ceres::UnitQuaternionRotatePoint(PatternRotationQuaternion, Result, Temp);
				Temp[0] += PatternTranslationVector[0];
				Temp[1] += PatternTranslationVector[1];
				Temp[2] += PatternTranslationVector[2];

				ceres::UnitQuaternionRotatePoint(CameraRotationQuaternion, Temp, Result);
				Result[0] += CameraTranslationVector[0];
				Result[1] += CameraTranslationVector[1];
				Result[2] += CameraTranslationVector[2];
			}
			template <typename T> void GetCameraCoordinate(
				const T* const PatternTranslationVector,
				const T* const PatternTranslationVectorMultiplicative,
				const T* const PatternRotationQuaternion,
				const T* const CameraTranslationVector,
				const T* const CameraRotationQuaternion,
				T* Result) const {

				T Multiplier = (T)AdditionalTranslationMultiplier;

				Result[0] = (T)PatternPoint[0] + Multiplier * PatternTranslationVectorMultiplicative[0];//Changed 1.2.2017
				Result[1] = (T)PatternPoint[1] + Multiplier * PatternTranslationVectorMultiplicative[1];
				Result[2] = (T)PatternPoint[2] + Multiplier * PatternTranslationVectorMultiplicative[2];
				T Temp[3];

				ceres::UnitQuaternionRotatePoint(PatternRotationQuaternion, Result, Temp);

				//T Multiplier = (T)AdditionalTranslationMultiplier;
				/*Temp[0] += PatternTranslationVector[0] + Multiplier * PatternTranslationVectorMultiplicative[0];
				Temp[1] += PatternTranslationVector[1] + Multiplier * PatternTranslationVectorMultiplicative[1];
				Temp[2] += PatternTranslationVector[2] + Multiplier * PatternTranslationVectorMultiplicative[2];*/

				Temp[0] += PatternTranslationVector[0];
				Temp[1] += PatternTranslationVector[1];
				Temp[2] += PatternTranslationVector[2];

				ceres::UnitQuaternionRotatePoint(CameraRotationQuaternion, Temp, Result);
				Result[0] += CameraTranslationVector[0];
				Result[1] += CameraTranslationVector[1];
				Result[2] += CameraTranslationVector[2];
			}

			template <typename T> void GetImageCoordinate(
				const T* const CameraFOV,
				const T* const CameraPrincipal,
				const T* const CameraDistortion,
				T* CameraCoordinate,
				T* ImageCoordinate) const {

				CameraCoordinate[2] = (CameraCoordinate[2] == (T)0.0) ? (T)1.0 : (T)1.0 / CameraCoordinate[2];

				CameraCoordinate[0] *= CameraCoordinate[2];
				CameraCoordinate[1] *= CameraCoordinate[2];

				{
					T r2, r4, r6, a1, a2, a3, cdist, icdist2;
					//double xd, yd, xd0, yd0, invProj;
					T xd0, yd0;
					/*Vec3d vecTilt;
					Vec3d dVecTilt;
					Matx22d dMatTilt;
					Vec2d dXdYd;*/

					T x = CameraCoordinate[0];
					T y = CameraCoordinate[1];

					r2 = x*x + y*y;
					r4 = r2*r2;
					r6 = r4*r2;
					a1 = (T)2.0 * x*y;
					a2 = r2 + (T)2.0 * x*x;
					a3 = r2 + (T)2.0 * y*y;

#define K(I) ((I < DistortionCoefficientsCount) ? CameraDistortion[I]: (T)0.0)

					cdist = (T)1.0 + K(0) * r2 + K(1) * r4 + K(4) * r6;
					icdist2 = (T)1.0 / ((T)1.0 + K(5) * r2 + K(6) * r4 + K(7) * r6);
					xd0 = x*cdist*icdist2 + K(2) * a1 + K(3) * a2 + K(8) * r2 + K(9) * r4;
					yd0 = y*cdist*icdist2 + K(2) * a3 + K(3) * a1 + K(10) * r2 + K(11) * r4;

#undef K
					// additional distortion by projecting onto a tilt plane
					/*vecTilt = matTilt*Vec3d(xd0, yd0, 1);
					invProj = vecTilt(2) ? 1. / vecTilt(2) : 1;
					xd = invProj * vecTilt(0);
					yd = invProj * vecTilt(1);*/

					ImageCoordinate[0] = xd0;
					ImageCoordinate[1] = yd0;
					/*m[i].x = xd*fx + cx;
					m[i].y = yd*fy + cy;*/
				}

				//DistortPoint(Temp, Result, CameraDistortion);

				ImageCoordinate[0] = ImageCoordinate[0] * CameraFOV[0] + CameraPrincipal[0];
				ImageCoordinate[1] = ImageCoordinate[1] * CameraFOV[1] + CameraPrincipal[1];
			}

			template <typename T> void GetNormalizedPoint(
				const T* const CameraFOV,
				const T* const CameraPrincipal,
				const T* const CameraDistortion,
				const T* const ImageCoordinate,
				T* NormalizedPoint,
				int iters = 5
				) const {
				T x = ImageCoordinate[0];
				T y = ImageCoordinate[1];
				x = (x - CameraPrincipal[0]) / CameraFOV[0];
				y = (y - CameraPrincipal[1]) / CameraFOV[1];


				T x0 = x;
				T y0 = y;
				//if (iters) {
				//    // compensate tilt distortion
				//    cv::Vec3d vecUntilt = invMatTilt * cv::Vec3d(x, y, 1);
				//    double invProj = vecUntilt(2) ? 1. / vecUntilt(2) : 1;
				//    x0 = x = invProj * vecUntilt(0);
				//    y0 = y = invProj * vecUntilt(1);
				//}

#define K(I) ((I < DistortionCoefficientsCount) ? CameraDistortion[I]: (T)0.0)
				// compensate distortion iteratively
				for (int j = 0; j < iters; j++) {
					T r2 = x*x + y*y;
					T icdist = ((T)1.0 + ((K(7) * r2 + K(6))*r2 + K(5))*r2) / ((T)1.0 + ((K(4) * r2 + K(1))*r2 + K(0))*r2);
					T deltaX = (T)2.0 * K(2) * x*y + K(3) * (r2 + (T)2.0 * x*x) + K(8) * r2 + K(9) * r2*r2;
					T deltaY = K(2) * (r2 + (T)2.0 * y*y) + (T)2.0 * K(3) * x*y + K(10) * r2 + K(11) * r2*r2;
					x = (x0 - deltaX)*icdist;
					y = (y0 - deltaY)*icdist;
				}
#undef K
				/*double xx = RR[0][0] * x + RR[0][1] * y + RR[0][2];
				double yy = RR[1][0] * x + RR[1][1] * y + RR[1][2];
				double ww = 1. / (RR[2][0] * x + RR[2][1] * y + RR[2][2]);
				x = xx*ww;
				y = yy*ww;*/

				/*if (dtype == CV_32FC2) {
				dstf[i*dstep].x = (float)x;
				dstf[i*dstep].y = (float)y;
				} else {
				dstd[i*dstep].x = x;
				dstd[i*dstep].y = y;
				}*/
				NormalizedPoint[0] = x;
				NormalizedPoint[1] = y;
				NormalizedPoint[2] = (T)1.0;

			}

			template <typename T> void MaintainCoordinate3D(const T* const Coordinate3D) const {}
			template <>
			void MaintainCoordinate3D(const double* const Coordinate3D) const {
				if (Coordinate3D) {
					this->Coordinate3D->x = Coordinate3D[0];
					this->Coordinate3D->y = Coordinate3D[1];
					this->Coordinate3D->z = Coordinate3D[2];
				}
			}

			template <typename T> void MaintainReprojectionError(const T* const ReprojectionError) const {}
			template <>
			void MaintainReprojectionError(const double* const ReprojectionError) const {
				if (ReprojectionError) {
					this->ReprojectionError->x = ReprojectionError[0];
					this->ReprojectionError->y = ReprojectionError[1];
				}
			}

			template <typename T> bool operator()(
				const T* const CameraIntrinsic,
				const T* const PatternTranslationVector,
				const T* const PatternRotationQuaternion,
				const T* const CameraTranslationVector,
				const T* const CameraRotationQuaternion,
				T* residual) const {

				T CameraCoordinate[3];
				T ImageCoordinate[2];
				GetCameraCoordinate(PatternTranslationVector, PatternRotationQuaternion, CameraTranslationVector, CameraRotationQuaternion, CameraCoordinate);
				MaintainCoordinate3D(CameraCoordinate);
				GetImageCoordinate(&CameraIntrinsic[0], &CameraIntrinsic[2], &CameraIntrinsic[4], CameraCoordinate, ImageCoordinate);
				//GetNormalizedPoint(CameraFOV, CameraPrincipal, CameraDistortion, ImageCoordinate, CameraCoordinate);

				residual[0] = ImageCoordinate[0] - (T)ImagePoint[0];
				residual[1] = ImageCoordinate[1] - (T)ImagePoint[1];

				MaintainReprojectionError(residual);

				return true;
			}
			template <typename T> bool operator()(
				const T* const CameraIntrinsic,
				const T* const PatternTranslationVector,
				const T* const PatternTranslationVectorMultiplicative,
				const T* const PatternRotationQuaternion,
				const T* const CameraTranslationVector,
				const T* const CameraRotationQuaternion,
				T* residual) const {

				T CameraCoordinate[3];
				T ImageCoordinate[2];
				GetCameraCoordinate(PatternTranslationVector, PatternTranslationVectorMultiplicative, PatternRotationQuaternion, CameraTranslationVector, CameraRotationQuaternion, CameraCoordinate);
				MaintainCoordinate3D(CameraCoordinate);
				GetImageCoordinate(&CameraIntrinsic[0], &CameraIntrinsic[2], &CameraIntrinsic[4], CameraCoordinate, ImageCoordinate);
				//GetNormalizedPoint(CameraFOV, CameraPrincipal, CameraDistortion, ImageCoordinate, CameraCoordinate);

				residual[0] = ImageCoordinate[0] - (T)ImagePoint[0];
				residual[1] = ImageCoordinate[1] - (T)ImagePoint[1];

				MaintainReprojectionError(residual);

				return true;
			}

			double PatternPoint[3];
			double ImagePoint[2];

			double AdditionalTranslationMultiplier;

			mutable cv::Point3d* Coordinate3D;
			mutable cv::Point2d* ReprojectionError;

			MeasuredCalibrationPointFunctor(cv::Point3d PatternPoint, cv::Point2d ImagePoint, double AdditionalTranslationMultiplier = 0.0) : Coordinate3D(NULL), ReprojectionError(NULL) {
				this->PatternPoint[0] = PatternPoint.x;
				this->PatternPoint[1] = PatternPoint.y;
				this->PatternPoint[2] = PatternPoint.z;

				this->ImagePoint[0] = ImagePoint.x;
				this->ImagePoint[1] = ImagePoint.y;

				this->AdditionalTranslationMultiplier = AdditionalTranslationMultiplier;

			}
			MeasuredCalibrationPointFunctor(cv::Point3d PatternPoint, cv::Point2d ImagePoint, double AdditionalTranslationMultiplier, cv::Point3d& Coordinate3D) : Coordinate3D(&Coordinate3D), ReprojectionError(NULL) {
				this->PatternPoint[0] = PatternPoint.x;
				this->PatternPoint[1] = PatternPoint.y;
				this->PatternPoint[2] = PatternPoint.z;

				this->ImagePoint[0] = ImagePoint.x;
				this->ImagePoint[1] = ImagePoint.y;

				this->AdditionalTranslationMultiplier = AdditionalTranslationMultiplier;

			}
			MeasuredCalibrationPointFunctor(cv::Point3d PatternPoint, cv::Point2d ImagePoint, double AdditionalTranslationMultiplier, cv::Point3d& Coordinate3D, cv::Point2d& ReprojectionError) : Coordinate3D(&Coordinate3D), ReprojectionError(&ReprojectionError) {
				this->PatternPoint[0] = PatternPoint.x;
				this->PatternPoint[1] = PatternPoint.y;
				this->PatternPoint[2] = PatternPoint.z;

				this->ImagePoint[0] = ImagePoint.x;
				this->ImagePoint[1] = ImagePoint.y;

				this->AdditionalTranslationMultiplier = AdditionalTranslationMultiplier;
			}
			virtual ~MeasuredCalibrationPointFunctor() {
			}

			static ceres::CostFunction* Create(cv::Point3d PatternPoint, cv::Point2d ImagePoint, cv::Point3d& Coordinate3D, cv::Point2d& ReprojectionError) {
				return (new ceres::AutoDiffCostFunction<MeasuredCalibrationPointFunctor, 2, 2 + 2 + DistortionCoefficientsCount, 3, 4, 3, 4>(new MeasuredCalibrationPointFunctor(PatternPoint, ImagePoint, 0.0, Coordinate3D, ReprojectionError)));
			}
			static ceres::CostFunction* Create(cv::Point3d PatternPoint, cv::Point2d ImagePoint, double AdditionalTranslationMultiplier, cv::Point3d& Coordinate3D, cv::Point2d& ReprojectionError) {
				return (new ceres::AutoDiffCostFunction<MeasuredCalibrationPointFunctor, 2, 2 + 2 + DistortionCoefficientsCount, 3, 3, 4, 3, 4>(new MeasuredCalibrationPointFunctor(PatternPoint, ImagePoint, AdditionalTranslationMultiplier, Coordinate3D, ReprojectionError)));
			}
		};


		//template <int DistortionCoefficientsCount, int Order>
		//class MeasuredCalibrationPointWithWarpFunctor : public MeasuredCalibrationPointFunctor <DistortionCoefficientsCount> {
		//public:
		//    template <typename T> bool operator() (
		//        /*const T* const CameraFOV,
		//        const T* const CameraPrincipal,
		//        const T* const CameraDistortion,*/
		//        const T* const CameraIntrinsic,
		//        const T* const PatternTranslationVector,
		//        const T* const PatternRotationQuaternion,
		//        const T* const CameraTranslationVector,
		//        const T* const CameraRotationQuaternion,
		//        const T* const BezierPoints,
		//        T* residual) const {

		//        T CameraCoordinate[3];
		//        T WarpedCoordinate[3];
		//        T ImageCoordinate[2];
		//        GetCameraCoordinate(PatternTranslationVector, PatternRotationQuaternion, CameraTranslationVector, CameraRotationQuaternion, CameraCoordinate);
		//        MaintainCoordinate3D(CameraCoordinate);
		//        Warper.WarpPoint(CameraCoordinate, WarpedCoordinate, BezierPoints);
		//        GetImageCoordinate(&CameraIntrinsic[0], &CameraIntrinsic[2], &CameraIntrinsic[4], WarpedCoordinate, ImageCoordinate);
		//        residual[0] = ImageCoordinate[0] - (T)ImagePoint[0];
		//        residual[1] = ImageCoordinate[1] - (T)ImagePoint[1];
		//        MaintainReprojectionError(residual);

		//        return true;
		//    }
		//    MeasuredCalibrationPointWithWarpFunctor(const cv::Point3d& PatternPoint, const cv::Point2d& ImagePoint, cv::Point3d& Coordinate3D, cv::Point2d& ReprojectionError, const MicroBezierWarper<Order + 1>& Warper) : MeasuredCalibrationPointFunctor(PatternPoint, ImagePoint, Coordinate3D, ReprojectionError), Warper(Warper) {
		//    }
		//    virtual ~MeasuredCalibrationPointWithWarpFunctor() {
		//    }
		//    static ceres::CostFunction* Create(const cv::Point3d& PatternPoint, const cv::Point2d& ImagePoint, cv::Point3d& Coordinate3D, cv::Point2d& ReprojectionError, const MicroBezierWarper<Order + 1>& Warper) {
		//        return (new ceres::AutoDiffCostFunction<MeasuredCalibrationPointWithWarpFunctor, 2, 2 + 2 + DistortionCoefficientsCount, 3, 4, 3, 4, (Order + 1) * (Order + 1) * (Order + 1) * 3>(new MeasuredCalibrationPointWithWarpFunctor(PatternPoint, ImagePoint, Coordinate3D, ReprojectionError, Warper)));
		//    }
		//    const MicroBezierWarper<Order + 1>& Warper;
		//};

		template <int DistortionCoefficientsCount, int Order>
		class MeasuredCalibrationPointWithDistortionWarpFunctor : public MeasuredCalibrationPointFunctor <DistortionCoefficientsCount> {
		public:
			template <typename T> bool operator() (
				const T* const CameraIntrinsic,
				const T* const PatternTranslationVector,
				const T* const PatternRotationQuaternion,
				const T* const CameraTranslationVector,
				const T* const CameraRotationQuaternion,
				T* residual) const {

				T CameraCoordinate[3];
				T ImageCoordinate[2];
				T DistortionCoefficients[DistortionCoefficientsCount];
				GetCameraCoordinate(PatternTranslationVector, PatternRotationQuaternion, CameraTranslationVector, CameraRotationQuaternion, CameraCoordinate);
				MaintainCoordinate3D(CameraCoordinate);
				Warper.GetDistiortion(&CameraIntrinsic[4], CameraCoordinate[2], DistortionCoefficients);

				GetImageCoordinate(&CameraIntrinsic[0], &CameraIntrinsic[2], &CameraIntrinsic[4], CameraCoordinate, ImageCoordinate);

				residual[0] = ImageCoordinate[0] - (T)ImagePoint[0];
				residual[1] = ImageCoordinate[1] - (T)ImagePoint[1];
				MaintainReprojectionError(residual);

				return true;
			}
			template <typename T> bool operator() (
				const T* const CameraIntrinsic,
				const T* const PatternTranslationVector,
				const T* const PatternTranslationVectorMultiplicative,
				const T* const PatternRotationQuaternion,
				const T* const CameraTranslationVector,
				const T* const CameraRotationQuaternion,
				T* residual) const {

				T CameraCoordinate[3];
				T ImageCoordinate[2];
				T DistortionCoefficients[DistortionCoefficientsCount];
				GetCameraCoordinate(PatternTranslationVector, PatternTranslationVectorMultiplicative, PatternRotationQuaternion, CameraTranslationVector, CameraRotationQuaternion, CameraCoordinate);
				MaintainCoordinate3D(CameraCoordinate);
				Warper.GetDistiortion(&CameraIntrinsic[4], CameraCoordinate[2], DistortionCoefficients);

				GetImageCoordinate(&CameraIntrinsic[0], &CameraIntrinsic[2], &CameraIntrinsic[4], CameraCoordinate, ImageCoordinate);

				residual[0] = ImageCoordinate[0] - (T)ImagePoint[0];
				residual[1] = ImageCoordinate[1] - (T)ImagePoint[1];
				MaintainReprojectionError(residual);

				return true;
			}
			MeasuredCalibrationPointWithDistortionWarpFunctor(const cv::Point3d& PatternPoint, const cv::Point2d& ImagePoint, cv::Point3d& Coordinate3D, cv::Point2d& ReprojectionError, const MicroBezierWarperDistortion<Order + 1, DistortionCoefficientsCount>& Warper, double AdditionalTranslationMultiplier = 0.0) : MeasuredCalibrationPointFunctor(PatternPoint, ImagePoint, AdditionalTranslationMultiplier, Coordinate3D, ReprojectionError), Warper(Warper) {
			}
			virtual ~MeasuredCalibrationPointWithDistortionWarpFunctor() {
			}
			static ceres::CostFunction* Create(const cv::Point3d& PatternPoint, const cv::Point2d& ImagePoint, cv::Point3d& Coordinate3D, cv::Point2d& ReprojectionError, const MicroBezierWarperDistortion<Order + 1, DistortionCoefficientsCount>& Warper) {
				return (new ceres::AutoDiffCostFunction<MeasuredCalibrationPointWithDistortionWarpFunctor, 2, 2 + 2 + (Order + 1) * DistortionCoefficientsCount, 3, 4, 3, 4>(new MeasuredCalibrationPointWithDistortionWarpFunctor(PatternPoint, ImagePoint, Coordinate3D, ReprojectionError, Warper)));
			}
			static ceres::CostFunction* Create(const cv::Point3d& PatternPoint, const cv::Point2d& ImagePoint, cv::Point3d& Coordinate3D, cv::Point2d& ReprojectionError, const MicroBezierWarperDistortion<Order + 1, DistortionCoefficientsCount>& Warper, double AdditionalTranslationMultiplier) {
				return (new ceres::AutoDiffCostFunction<MeasuredCalibrationPointWithDistortionWarpFunctor, 2, 2 + 2 + (Order + 1) * DistortionCoefficientsCount, 3, 3, 4, 3, 4>(new MeasuredCalibrationPointWithDistortionWarpFunctor(PatternPoint, ImagePoint, Coordinate3D, ReprojectionError, Warper, AdditionalTranslationMultiplier)));
			}
			const MicroBezierWarperDistortion<Order + 1, DistortionCoefficientsCount>& Warper;
		};

		template <int DistortionCoefficientsCount, int Order>
		class MeasuredCalibrationPointWithDistortionWarpReversedFunctor : public MeasuredCalibrationPointFunctor <DistortionCoefficientsCount> {
		public:
			/*template <typename T> void MaintainCoordinate3D(const T* const Coordinate3D) const {}
			template <>
			void MaintainCoordinate3D(const double* const Coordinate3D) const {
			if (Coordinate3D) {
			this->Coordinate3D->x = Coordinate3D[0];
			this->Coordinate3D->y = Coordinate3D[1];
			this->Coordinate3D->z = Coordinate3D[2];
			}
			}*/

			template <typename T> void MaintainReprojectionError3D(const T* const ReprojectionError) const {}
			template <>
			void MaintainReprojectionError3D(const double* const ReprojectionError3D) const {
				if (ReprojectionError3D) {
					this->ReprojectionError3D->x = ReprojectionError3D[0];
					this->ReprojectionError3D->y = ReprojectionError3D[1];
					this->ReprojectionError3D->z = ReprojectionError3D[2];
				}
			}

			template <typename T> bool LineLineIntersection(const T* const BasePoint0, const T* const Direction0, const T* const BasePoint1, const T* const Direction1, T* T0, T* T1) const {
				T BasePointsVector[3] = { BasePoint0[0] - BasePoint1[0], BasePoint0[1] - BasePoint1[1], BasePoint0[2] - BasePoint1[2] };
				//cv::Point3_ <T> BasePointsVector = BasePoint - SecondLine.BasePoint;
				T d1343, d4321, d1321, d4343, d2121;
				T Numer, Denom;

#define DOT_MACRO(V0, V1) V0[0] * V1[0] + V0[1] * V1[1] + V0[2] * V1[2];
				//d1343 = BasePointsVector.dot(SecondLine.Direction);
				d1343 = DOT_MACRO(BasePointsVector, Direction1);
				//d4321 = SecondLine.Direction.dot(Direction);
				d4321 = DOT_MACRO(Direction1, Direction0);
				//d1321 = BasePointsVector.dot(Direction);
				d1321 = DOT_MACRO(BasePointsVector, Direction0);
				//d4343 = SecondLine.Direction.dot(SecondLine.Direction);
				d4343 = DOT_MACRO(Direction1, Direction1);
				//d2121 = Direction.dot(Direction);
				d2121 = DOT_MACRO(Direction0, Direction0);
#undef  DOT_MACRO

				Denom = d2121 * d4343 - d4321 * d4321;
				if (ceres::abs(Denom) < (T)FLT_EPSILON) return false;
				Numer = d1343 * d4321 - d1321 * d4343;

				*T0 = Numer / Denom;
				*T1 = (d1343 + d4321 * (*T0)) / d4343;
				return true;
			}
			template <typename T> bool operator() (
				const T* const Camera0Intrinsic,
				const T* const Camera1Intrinsic,
				const T* const PatternTranslationVector,
				const T* const PatternTranslationVectorMultiplicative,
				const T* const PatternRotationQuaternion,
				const T* const Camera1TranslationVector,
				const T* const Camera1RotationQuaternion,
				T* residual) const {
				T ImageCoordinate0[2] = { (T)ImagePoint[0], (T)ImagePoint[1] };
				T ImageCoordinate1[2] = { (T)ImagePoint1[0], (T)ImagePoint1[1] };

				T Direction0[3];
				T Direction1[3];

				T CameraCoordinate0[3];
				T CameraCoordinate1[3];

				T DistortionCoefficients0[DistortionCoefficientsCount];
				T DistortionCoefficients1[DistortionCoefficientsCount];

				T Camera0TranslationVector[3] = { (T)0.0, (T)0.0, (T)0.0 };
				T Camera0RotationQuaternion[4] = { (T)1.0, (T)0.0, (T)0.0, (T)0.0 };

				T TranslationDirection[3] = { PatternTranslationVectorMultiplicative[0], PatternTranslationVectorMultiplicative[1], (T)0.0 };

				if (UseTranslationMultiplicativeAsDirection) {
					TranslationDirection[2] = ITERATIVE_TRANSLATION_VECTOR_MULTIPLIER * ceres::sqrt((T)1.0 - PatternTranslationVectorMultiplicative[0] * PatternTranslationVectorMultiplicative[0] - PatternTranslationVectorMultiplicative[1] * PatternTranslationVectorMultiplicative[1]);
				}
				else {
					TranslationDirection[2] = PatternTranslationVectorMultiplicative[2];
				}

				GetCameraCoordinate(PatternTranslationVector, TranslationDirection, PatternRotationQuaternion, Camera0TranslationVector, Camera0RotationQuaternion, CameraCoordinate0);
				GetCameraCoordinate(PatternTranslationVector, TranslationDirection, PatternRotationQuaternion, Camera1TranslationVector, Camera1RotationQuaternion, CameraCoordinate1);

				Warper0.GetDistiortion(&Camera0Intrinsic[4], CameraCoordinate0[2], DistortionCoefficients0);
				Warper1.GetDistiortion(&Camera1Intrinsic[4], CameraCoordinate1[2], DistortionCoefficients1);

				T Camera0RotationQuaternionInv[4] = { Camera0RotationQuaternion[0], -Camera0RotationQuaternion[1], -Camera0RotationQuaternion[2], -Camera0RotationQuaternion[3] };
				T Camera1RotationQuaternionInv[4] = { Camera1RotationQuaternion[0], -Camera1RotationQuaternion[1], -Camera1RotationQuaternion[2], -Camera1RotationQuaternion[3] };

				T Temp[3] = { (T)0.0, (T)0.0, (T)0.0 };

				T Base0[3];
				T Base1[3];

				Temp[0] = -Camera0TranslationVector[0];
				Temp[1] = -Camera0TranslationVector[1];
				Temp[2] = -Camera0TranslationVector[2];

				ceres::UnitQuaternionRotatePoint(Camera0RotationQuaternionInv, Temp, Base0);

				Temp[0] = -Camera1TranslationVector[0];
				Temp[1] = -Camera1TranslationVector[1];
				Temp[2] = -Camera1TranslationVector[2];
				ceres::UnitQuaternionRotatePoint(Camera1RotationQuaternionInv, Temp, Base1);

				GetNormalizedPoint(&Camera0Intrinsic[0], &Camera0Intrinsic[2], DistortionCoefficients0, ImageCoordinate0, Temp);
				ceres::UnitQuaternionRotatePoint(Camera0RotationQuaternionInv, Temp, Direction0);

				GetNormalizedPoint(&Camera1Intrinsic[0], &Camera1Intrinsic[2], DistortionCoefficients1, ImageCoordinate1, Temp);
				ceres::UnitQuaternionRotatePoint(Camera1RotationQuaternionInv, Temp, Direction1);

				T PatternPointWorld[3];

				Temp[0] = (T)PatternPoint[0];
				Temp[1] = (T)PatternPoint[1];
				Temp[2] = (T)PatternPoint[2];

				ceres::UnitQuaternionRotatePoint(PatternRotationQuaternion, Temp, PatternPointWorld);

				T Multiplier = (T)AdditionalTranslationMultiplier;
				PatternPointWorld[0] += PatternTranslationVector[0] + Multiplier * TranslationDirection[0];
				PatternPointWorld[1] += PatternTranslationVector[1] + Multiplier * TranslationDirection[1];
				PatternPointWorld[2] += PatternTranslationVector[2] + Multiplier * TranslationDirection[2];

				T T0, T1;

				LineLineIntersection(Base0, Direction0, Base1, Direction1, &T0, &T1);

				T Point0[3] = { Base0[0] + Direction0[0] * T0, Base0[1] + Direction0[1] * T0, Base0[2] + Direction0[2] * T0 };
				T Point1[3] = { Base1[0] + Direction1[0] * T1, Base1[1] + Direction1[1] * T1, Base1[2] + Direction1[2] * T1 };

				T PointDifference[3] = { Point0[0] - Point1[0], Point0[1] - Point1[1], Point0[2] - Point1[2] };

				T Length = ceres::sqrt(PointDifference[0] * PointDifference[0] + PointDifference[1] * PointDifference[1] + PointDifference[2] * PointDifference[2]);

				T Half = (T)0.5;

				residual[0] = PatternPointWorld[0] - (Point0[0] + Point1[0]) * Half;
				residual[1] = PatternPointWorld[1] - (Point0[1] + Point1[1]) * Half;
				residual[2] = PatternPointWorld[2] - (Point0[2] + Point1[2]) * Half;

				MaintainCoordinate3D(&PatternPointWorld[0]);
				MaintainReprojectionError3D(residual);

				/*residual[0] *= Length;
				residual[1] *= Length;
				residual[2] *= Length;*/

				/*T TranslationLength = ceres::sqrt(PatternTranslationVectorMultiplicative[0] * PatternTranslationVectorMultiplicative[0] + PatternTranslationVectorMultiplicative[1] * PatternTranslationVectorMultiplicative[1] + PatternTranslationVectorMultiplicative[2] * PatternTranslationVectorMultiplicative[2]);
				if (TranslationLength > (T)0.0) {
				residual[0] /= TranslationLength;
				residual[1] /= TranslationLength;
				residual[2] /= TranslationLength;
				}*/

				return true;
			}
			template <typename T> bool operator() (
				const T* const Camera0Intrinsic,
				const T* const Camera1Intrinsic,
				const T* const PatternTranslationVector,
				const T* const PatternRotationQuaternion,
				const T* const Camera1TranslationVector,
				const T* const Camera1RotationQuaternion,
				T* residual) const {
				T TranslationMultiplicative[3] = { (T)0.0, (T)0.0, (T)0.0 };
				return this->operator()(Camera0Intrinsic, Camera1Intrinsic, PatternTranslationVector, TranslationMultiplicative, PatternRotationQuaternion, Camera1TranslationVector, Camera1RotationQuaternion, residual);
			}
			MeasuredCalibrationPointWithDistortionWarpReversedFunctor(const cv::Point3d& PatternPoint, const cv::Point2d& ImagePoint0, const cv::Point2d& ImagePoint1,
				cv::Point3d& Coordinate3D, cv::Point3d& ReprojectionError3D,
				const MicroBezierWarperDistortion<Order + 1, DistortionCoefficientsCount>& Warper0,
				const MicroBezierWarperDistortion<Order + 1, DistortionCoefficientsCount>& Warper1,
				double AdditionalTranslationMultiplier = 0.0,
				bool UseTranslationMultiplicativeAsDirection = false) : MeasuredCalibrationPointFunctor(PatternPoint, ImagePoint0, AdditionalTranslationMultiplier, Coordinate3D), Warper0(Warper0), Warper1(Warper1),
				ReprojectionError3D(&ReprojectionError3D), UseTranslationMultiplicativeAsDirection(UseTranslationMultiplicativeAsDirection) {
				this->ImagePoint1[0] = ImagePoint1.x;
				this->ImagePoint1[1] = ImagePoint1.y;
			}
			virtual ~MeasuredCalibrationPointWithDistortionWarpReversedFunctor() {
			}
			static ceres::CostFunction* Create(const cv::Point3d& PatternPoint, const cv::Point2d& ImagePoint0, const cv::Point2d& ImagePoint1, cv::Point3d& Coordinate3D, cv::Point3d& ReprojectionError3D, const MicroBezierWarperDistortion<Order + 1, DistortionCoefficientsCount>& Warper0, const MicroBezierWarperDistortion<Order + 1, DistortionCoefficientsCount>& Warper1) {
				return (new ceres::AutoDiffCostFunction<MeasuredCalibrationPointWithDistortionWarpReversedFunctor, 3, 2 + 2 + (Order + 1) * DistortionCoefficientsCount, 2 + 2 + (Order + 1) * DistortionCoefficientsCount, 3, 4, 3, 4>(new MeasuredCalibrationPointWithDistortionWarpReversedFunctor(PatternPoint, ImagePoint0, ImagePoint1, Coordinate3D, ReprojectionError3D, Warper0, Warper1)));
			}
			static ceres::CostFunction* CreateBoundedTranslation(const cv::Point3d& PatternPoint, const cv::Point2d& ImagePoint0, const cv::Point2d& ImagePoint1, cv::Point3d& Coordinate3D, cv::Point3d& ReprojectionError3D, const MicroBezierWarperDistortion<Order + 1, DistortionCoefficientsCount>& Warper0, const MicroBezierWarperDistortion<Order + 1, DistortionCoefficientsCount>& Warper1, double AdditionalTranslationMultiplier) {
				return (new ceres::AutoDiffCostFunction<MeasuredCalibrationPointWithDistortionWarpReversedFunctor, 3, 2 + 2 + (Order + 1) * DistortionCoefficientsCount, 2 + 2 + (Order + 1) * DistortionCoefficientsCount, 3, 3, 4, 3, 4>(new MeasuredCalibrationPointWithDistortionWarpReversedFunctor(PatternPoint, ImagePoint0, ImagePoint1, Coordinate3D, ReprojectionError3D, Warper0, Warper1, AdditionalTranslationMultiplier)));
			}
			static ceres::CostFunction* CreateBoundedTranslationOrientation(const cv::Point3d& PatternPoint, const cv::Point2d& ImagePoint0, const cv::Point2d& ImagePoint1, cv::Point3d& Coordinate3D, cv::Point3d& ReprojectionError3D, const MicroBezierWarperDistortion<Order + 1, DistortionCoefficientsCount>& Warper0, const MicroBezierWarperDistortion<Order + 1, DistortionCoefficientsCount>& Warper1, double AdditionalTranslationMultiplier) {
				return (new ceres::AutoDiffCostFunction<MeasuredCalibrationPointWithDistortionWarpReversedFunctor, 3, 2 + 2 + (Order + 1) * DistortionCoefficientsCount, 2 + 2 + (Order + 1) * DistortionCoefficientsCount, 3, 2, 4, 3, 4>(new MeasuredCalibrationPointWithDistortionWarpReversedFunctor(PatternPoint, ImagePoint0, ImagePoint1, Coordinate3D, ReprojectionError3D, Warper0, Warper1, AdditionalTranslationMultiplier, true)));
			}
			const MicroBezierWarperDistortion<Order + 1, DistortionCoefficientsCount>& Warper0;
			const MicroBezierWarperDistortion<Order + 1, DistortionCoefficientsCount>& Warper1;
			//double ImagePoint0[2];
			double ImagePoint1[2];


			//mutable cv::Point3d& Coordinate3D;
			mutable cv::Point3d* ReprojectionError3D;
			bool UseTranslationMultiplicativeAsDirection;
		};

		template <int DistortionCoefficientsCount, int Order>
		class MeasuredCalibrationPointWithIntrinsicWarpReversedFunctor : public MeasuredCalibrationPointFunctor <DistortionCoefficientsCount> {
		public:
			/*template <typename T> void MaintainCoordinate3D(const T* const Coordinate3D) const {}
			template <>
			void MaintainCoordinate3D(const double* const Coordinate3D) const {
			if (Coordinate3D) {
			this->Coordinate3D->x = Coordinate3D[0];
			this->Coordinate3D->y = Coordinate3D[1];
			this->Coordinate3D->z = Coordinate3D[2];
			}
			}*/

			template <typename T> void MaintainReprojectionError3D(const T* const ReprojectionError) const {}
			template <>
			void MaintainReprojectionError3D(const double* const ReprojectionError3D) const {
				if (ReprojectionError3D) {
					this->ReprojectionError3D->x = ReprojectionError3D[0];
					this->ReprojectionError3D->y = ReprojectionError3D[1];
					this->ReprojectionError3D->z = ReprojectionError3D[2];
				}
			}


			template <typename T> void MaintainNearestPointLineVector3D(const T* const NearestPointLineVector3D) const {}
			template <>
			void MaintainNearestPointLineVector3D(const double* const NearestPointLineVector3D) const {
				if (NearestPointLineVector3D) {
					this->NearestPointLineVector3D->x = NearestPointLineVector3D[0];
					this->NearestPointLineVector3D->y = NearestPointLineVector3D[1];
					this->NearestPointLineVector3D->z = NearestPointLineVector3D[2];
				}
			}

			template <typename T> bool LineLineIntersection(const T* const BasePoint0, const T* const Direction0, const T* const BasePoint1, const T* const Direction1, T* T0, T* T1) const {
				T BasePointsVector[3] = { BasePoint0[0] - BasePoint1[0], BasePoint0[1] - BasePoint1[1], BasePoint0[2] - BasePoint1[2] };
				//cv::Point3_ <T> BasePointsVector = BasePoint - SecondLine.BasePoint;
				T d1343, d4321, d1321, d4343, d2121;
				T Numer, Denom;

#define DOT_MACRO(V0, V1) V0[0] * V1[0] + V0[1] * V1[1] + V0[2] * V1[2];
				//d1343 = BasePointsVector.dot(SecondLine.Direction);
				d1343 = DOT_MACRO(BasePointsVector, Direction1);
				//d4321 = SecondLine.Direction.dot(Direction);
				d4321 = DOT_MACRO(Direction1, Direction0);
				//d1321 = BasePointsVector.dot(Direction);
				d1321 = DOT_MACRO(BasePointsVector, Direction0);
				//d4343 = SecondLine.Direction.dot(SecondLine.Direction);
				d4343 = DOT_MACRO(Direction1, Direction1);
				//d2121 = Direction.dot(Direction);
				d2121 = DOT_MACRO(Direction0, Direction0);
#undef  DOT_MACRO

				Denom = d2121 * d4343 - d4321 * d4321;
				if (ceres::abs(Denom) < (T)FLT_EPSILON) return false;
				Numer = d1343 * d4321 - d1321 * d4343;

				*T0 = Numer / Denom;
				*T1 = (d1343 + d4321 * (*T0)) / d4343;
				return true;
			}
			template <typename T> bool operator() (
				const T* const Camera0Intrinsic,
				const T* const Camera1Intrinsic,
				const T* const PatternTranslationVector,
				const T* const PatternTranslationVectorMultiplicative,
				const T* const PatternRotationQuaternion,
				const T* const Camera1TranslationVector,
				const T* const Camera1RotationQuaternion,
				T* residual) const {
				T ImageCoordinate0[2] = { (T)ImagePoint[0], (T)ImagePoint[1] };
				T ImageCoordinate1[2] = { (T)ImagePoint1[0], (T)ImagePoint1[1] };

				T Direction0[3];
				T Direction1[3];

				T CameraCoordinate0[3];
				T CameraCoordinate1[3];

				T Camera0IntrinsicDepth[4 + DistortionCoefficientsCount];
				T Camera1IntrinsicDepth[4 + DistortionCoefficientsCount];

				T Camera0TranslationVector[3] = { (T)0.0, (T)0.0, (T)0.0 };
				T Camera0RotationQuaternion[4] = { (T)1.0, (T)0.0, (T)0.0, (T)0.0 };

				T TranslationDirection[3] = { PatternTranslationVectorMultiplicative[0], PatternTranslationVectorMultiplicative[1], (T)0.0 };

				if (UseTranslationMultiplicativeAsDirection) {
					TranslationDirection[2] = ITERATIVE_TRANSLATION_VECTOR_MULTIPLIER * ceres::sqrt((T)1.0 - PatternTranslationVectorMultiplicative[0] * PatternTranslationVectorMultiplicative[0] - PatternTranslationVectorMultiplicative[1] * PatternTranslationVectorMultiplicative[1]);
				}
				else {
					TranslationDirection[2] = PatternTranslationVectorMultiplicative[2];
				}

				GetCameraCoordinate(PatternTranslationVector, TranslationDirection, PatternRotationQuaternion, Camera0TranslationVector, Camera0RotationQuaternion, CameraCoordinate0);
				GetCameraCoordinate(PatternTranslationVector, TranslationDirection, PatternRotationQuaternion, Camera1TranslationVector, Camera1RotationQuaternion, CameraCoordinate1);

				Warper0.GetIntrinsic(Camera0Intrinsic, CameraCoordinate0[2], Camera0IntrinsicDepth, FixFOV, FixPrincipal);
				Warper1.GetIntrinsic(Camera1Intrinsic, CameraCoordinate1[2], Camera1IntrinsicDepth, FixFOV, FixPrincipal);

				T Camera0RotationQuaternionInv[4] = { Camera0RotationQuaternion[0], -Camera0RotationQuaternion[1], -Camera0RotationQuaternion[2], -Camera0RotationQuaternion[3] };
				T Camera1RotationQuaternionInv[4] = { Camera1RotationQuaternion[0], -Camera1RotationQuaternion[1], -Camera1RotationQuaternion[2], -Camera1RotationQuaternion[3] };

				T Temp[3] = { (T)0.0, (T)0.0, (T)0.0 };

				T Base0[3];
				T Base1[3];

				Temp[0] = -Camera0TranslationVector[0];
				Temp[1] = -Camera0TranslationVector[1];
				Temp[2] = -Camera0TranslationVector[2];

				ceres::UnitQuaternionRotatePoint(Camera0RotationQuaternionInv, Temp, Base0);

				Temp[0] = -Camera1TranslationVector[0];
				Temp[1] = -Camera1TranslationVector[1];
				Temp[2] = -Camera1TranslationVector[2];
				ceres::UnitQuaternionRotatePoint(Camera1RotationQuaternionInv, Temp, Base1);

				GetNormalizedPoint(&Camera0IntrinsicDepth[0], &Camera0IntrinsicDepth[2], &Camera0IntrinsicDepth[4], ImageCoordinate0, Temp);
				ceres::UnitQuaternionRotatePoint(Camera0RotationQuaternionInv, Temp, Direction0);

				GetNormalizedPoint(&Camera1IntrinsicDepth[0], &Camera1IntrinsicDepth[2], &Camera1IntrinsicDepth[4], ImageCoordinate1, Temp);
				ceres::UnitQuaternionRotatePoint(Camera1RotationQuaternionInv, Temp, Direction1);

				T PatternPointWorld[3];

				/*Temp[0] = (T)PatternPoint[0];
				Temp[1] = (T)PatternPoint[1];
				Temp[2] = (T)PatternPoint[2];*/

				T Multiplier = (T)AdditionalTranslationMultiplier;
				Temp[0] = (T)PatternPoint[0] + Multiplier * TranslationDirection[0];
				Temp[1] = (T)PatternPoint[1] + Multiplier * TranslationDirection[1];
				Temp[2] = (T)PatternPoint[2] + Multiplier * TranslationDirection[2];

				ceres::UnitQuaternionRotatePoint(PatternRotationQuaternion, Temp, PatternPointWorld);

				/*T Multiplier = (T)AdditionalTranslationMultiplier;
				PatternPointWorld[0] += PatternTranslationVector[0] + Multiplier * TranslationDirection[0];
				PatternPointWorld[1] += PatternTranslationVector[1] + Multiplier * TranslationDirection[1];
				PatternPointWorld[2] += PatternTranslationVector[2] + Multiplier * TranslationDirection[2];*/

				PatternPointWorld[0] += PatternTranslationVector[0];
				PatternPointWorld[1] += PatternTranslationVector[1];
				PatternPointWorld[2] += PatternTranslationVector[2];


				T T0, T1;

				LineLineIntersection(Base0, Direction0, Base1, Direction1, &T0, &T1);

				T Point0[3] = { Base0[0] + Direction0[0] * T0, Base0[1] + Direction0[1] * T0, Base0[2] + Direction0[2] * T0 };
				T Point1[3] = { Base1[0] + Direction1[0] * T1, Base1[1] + Direction1[1] * T1, Base1[2] + Direction1[2] * T1 };

				T PointDifference[3] = { Point0[0] - Point1[0], Point0[1] - Point1[1], Point0[2] - Point1[2] };

				MaintainNearestPointLineVector3D(PointDifference);

				T Length = ceres::sqrt(PointDifference[0] * PointDifference[0] + PointDifference[1] * PointDifference[1] + PointDifference[2] * PointDifference[2]);

				T Half = (T)0.5;

				residual[0] = PatternPointWorld[0] - (Point0[0] + Point1[0]) * Half;
				residual[1] = PatternPointWorld[1] - (Point0[1] + Point1[1]) * Half;
				residual[2] = PatternPointWorld[2] - (Point0[2] + Point1[2]) * Half;

				/*residual[0] = PatternPointWorld[0] - (Point0[0] + Point1[0]) * Half;
				residual[1] = PatternPointWorld[1] - (Point0[1] + Point1[1]) * Half;
				residual[2] = PatternPointWorld[2] - (Point0[2] + Point1[2]) * Half;*/

				MaintainCoordinate3D(&PatternPointWorld[0]);
				MaintainReprojectionError3D(residual);

				Point0[0] = PatternPointWorld[0] - Point0[0];
				Point0[1] = PatternPointWorld[1] - Point0[1];
				Point0[2] = PatternPointWorld[2] - Point0[2];

				Point1[0] = PatternPointWorld[0] - Point1[0];
				Point1[1] = PatternPointWorld[1] - Point1[1];
				Point1[2] = PatternPointWorld[2] - Point1[2];

				T Point0LengthS = Point0[0] * Point0[0] + Point0[1] * Point0[1] + Point0[2] * Point0[2];
				T Point1LengthS = Point1[0] * Point1[0] + Point1[1] * Point1[1] + Point1[2] * Point1[2];

				if (Point0LengthS > Point1LengthS) {
					residual[0] = Point0[0];
					residual[1] = Point0[1];
					residual[2] = Point0[2];
				}
				else {
					residual[0] = Point1[0];
					residual[1] = Point1[1];
					residual[2] = Point1[2];
				}

				/*T ResidualLength = ceres::sqrt(residual[0] * residual[0] + residual[1] * residual[1] + residual[2] * residual[2]);
				Multiplier = (T)1.0;
				if (ResidualLength > (T)0.00001) {
				Multiplier = (ResidualLength + Length) / ResidualLength;
				}
				residual[0] *= Multiplier;
				residual[1] *= Multiplier;
				residual[2] *= Multiplier;*/

				/*residual[0] *= Length;
				residual[1] *= Length;
				residual[2] *= Length;*/

				/*T TranslationLength = ceres::sqrt(PatternTranslationVectorMultiplicative[0] * PatternTranslationVectorMultiplicative[0] + PatternTranslationVectorMultiplicative[1] * PatternTranslationVectorMultiplicative[1] + PatternTranslationVectorMultiplicative[2] * PatternTranslationVectorMultiplicative[2]);
				if (TranslationLength > (T)0.0) {
				residual[0] /= TranslationLength;
				residual[1] /= TranslationLength;
				residual[2] /= TranslationLength;
				}*/

				return true;
			}
			template <typename T> bool operator() (
				const T* const Camera0Intrinsic,
				const T* const Camera1Intrinsic,
				const T* const PatternTranslationVector,
				const T* const PatternRotationQuaternion,
				const T* const Camera1TranslationVector,
				const T* const Camera1RotationQuaternion,
				T* residual) const {
				T TranslationMultiplicative[3] = { (T)0.0, (T)0.0, (T)0.0 };
				return this->operator()(Camera0Intrinsic, Camera1Intrinsic, PatternTranslationVector, TranslationMultiplicative, PatternRotationQuaternion, Camera1TranslationVector, Camera1RotationQuaternion, residual);
			}
			MeasuredCalibrationPointWithIntrinsicWarpReversedFunctor(const cv::Point3d& PatternPoint, const cv::Point2d& ImagePoint0, const cv::Point2d& ImagePoint1,
				cv::Point3d& Coordinate3D, cv::Point3d& ReprojectionError3D, cv::Point3d& NearestPointLineVector3D,
				const MicroBezierWarperIntrinsic<Order + 1, DistortionCoefficientsCount>& Warper0,
				const MicroBezierWarperIntrinsic<Order + 1, DistortionCoefficientsCount>& Warper1,
				double AdditionalTranslationMultiplier = 0.0,
				bool UseTranslationMultiplicativeAsDirection = false) : MeasuredCalibrationPointFunctor(PatternPoint, ImagePoint0, AdditionalTranslationMultiplier, Coordinate3D), Warper0(Warper0), Warper1(Warper1),
				ReprojectionError3D(&ReprojectionError3D), NearestPointLineVector3D(&NearestPointLineVector3D), UseTranslationMultiplicativeAsDirection(UseTranslationMultiplicativeAsDirection) {
				this->ImagePoint1[0] = ImagePoint1.x;
				this->ImagePoint1[1] = ImagePoint1.y;
				FixFOV = false;
				FixPrincipal = false;
			}
			virtual ~MeasuredCalibrationPointWithIntrinsicWarpReversedFunctor() {
			}
			static ceres::CostFunction* Create(const cv::Point3d& PatternPoint, const cv::Point2d& ImagePoint0, const cv::Point2d& ImagePoint1, cv::Point3d& Coordinate3D, cv::Point3d& ReprojectionError3D, cv::Point3d& NearestPointLineVector3D, const MicroBezierWarperIntrinsic<Order + 1, DistortionCoefficientsCount>& Warper0, const MicroBezierWarperIntrinsic<Order + 1, DistortionCoefficientsCount>& Warper1) {
				return (new ceres::AutoDiffCostFunction<MeasuredCalibrationPointWithIntrinsicWarpReversedFunctor, 3, (Order + 1) * (2 + 2 + DistortionCoefficientsCount), (Order + 1) * (2 + 2 + DistortionCoefficientsCount), 3, 4, 3, 4>(new MeasuredCalibrationPointWithIntrinsicWarpReversedFunctor(PatternPoint, ImagePoint0, ImagePoint1, Coordinate3D, ReprojectionError3D, NearestPointLineVector3D, Warper0, Warper1)));
			}
			static ceres::CostFunction* CreateBoundedTranslation(const cv::Point3d& PatternPoint, const cv::Point2d& ImagePoint0, const cv::Point2d& ImagePoint1, cv::Point3d& Coordinate3D, cv::Point3d& ReprojectionError3D, cv::Point3d& NearestPointLineVector3D, const MicroBezierWarperIntrinsic<Order + 1, DistortionCoefficientsCount>& Warper0, const MicroBezierWarperIntrinsic<Order + 1, DistortionCoefficientsCount>& Warper1, double AdditionalTranslationMultiplier) {
				return (new ceres::AutoDiffCostFunction<MeasuredCalibrationPointWithIntrinsicWarpReversedFunctor, 3, (Order + 1) * (2 + 2 + DistortionCoefficientsCount), (Order + 1) * (2 + 2 + DistortionCoefficientsCount), 3, 3, 4, 3, 4>(new MeasuredCalibrationPointWithIntrinsicWarpReversedFunctor(PatternPoint, ImagePoint0, ImagePoint1, Coordinate3D, ReprojectionError3D, NearestPointLineVector3D, Warper0, Warper1, AdditionalTranslationMultiplier)));
			}
			static ceres::CostFunction* CreateBoundedTranslationOrientation(const cv::Point3d& PatternPoint, const cv::Point2d& ImagePoint0, const cv::Point2d& ImagePoint1, cv::Point3d& Coordinate3D, cv::Point3d& ReprojectionError3D, cv::Point3d& NearestPointLineVector3D, const MicroBezierWarperIntrinsic<Order + 1, DistortionCoefficientsCount>& Warper0, const MicroBezierWarperIntrinsic<Order + 1, DistortionCoefficientsCount>& Warper1, double AdditionalTranslationMultiplier) {
				return (new ceres::AutoDiffCostFunction<MeasuredCalibrationPointWithIntrinsicWarpReversedFunctor, 3, (Order + 1) * (2 + 2 + DistortionCoefficientsCount), (Order + 1) * (2 + 2 + DistortionCoefficientsCount), 3, 2, 4, 3, 4>(new MeasuredCalibrationPointWithIntrinsicWarpReversedFunctor(PatternPoint, ImagePoint0, ImagePoint1, Coordinate3D, ReprojectionError3D, NearestPointLineVector3D, Warper0, Warper1, AdditionalTranslationMultiplier, true)));
			}
			const MicroBezierWarperIntrinsic<Order + 1, DistortionCoefficientsCount>& Warper0;
			const MicroBezierWarperIntrinsic<Order + 1, DistortionCoefficientsCount>& Warper1;
			//double ImagePoint0[2];
			double ImagePoint1[2];


			//mutable cv::Point3d& Coordinate3D;
			mutable cv::Point3d* ReprojectionError3D;
			mutable cv::Point3d* NearestPointLineVector3D;
			bool UseTranslationMultiplicativeAsDirection;
			bool FixFOV, FixPrincipal;
		};


		bool CompareCorrespondencePair(const std::pair<cv::Point2d, cv::Point3d>& first, const std::pair<cv::Point2d, cv::Point3d>& second) {
			if (first.second.z < second.second.z) return true;
			if (first.second.z > second.second.z) return false;
			if (first.second.y < second.second.y) return true;
			if (first.second.y > second.second.y) return false;
			if (first.second.x < second.second.x) return true;
			if (first.second.x >= second.second.x) return false;
			return false;
		}
		bool CompareCorrespondencePair2(const std::pair<cv::Point3d, cv::Point3d>& first, const std::pair<cv::Point3d, cv::Point3d>& second) {
			if (first.second.z < second.second.z) return true;
			if (first.second.z > second.second.z) return false;
			if (first.second.y < second.second.y) return true;
			if (first.second.y > second.second.y) return false;
			if (first.second.x < second.second.x) return true;
			if (first.second.x >= second.second.x) return false;
			return false;
		}

		bool CompareCorrespondencePattern(const std::pair<int, std::vector<std::pair<cv::Point2d, cv::Point3d>>>& first, const std::pair<int, std::vector<std::pair<cv::Point2d, cv::Point3d>>>& second) {
			return (first.first < second.first);
		}
		bool CompareCorrespondencePattern2(const std::pair<int, std::vector<std::pair<cv::Point3d, cv::Point3d>>>& first, const std::pair<int, std::vector<std::pair<cv::Point3d, cv::Point3d>>>& second) {
			return (first.first < second.first);
		}

		/*   template <typename T, typename K>
		struct VectorBoxConvertor {
		static void Convert(std::vector<T>& Input, vectorBox<K>& Output) {
		for (std::size_t i = 0; i < Input.size(); ++i) {
		T& InputElement = Input[i];
		K OutputElement;
		Convert(InputElement, OutputElement);
		}
		}
		};

		template <typename T>
		struct VectorBoxConvertor<T, T> {
		void Convert(T& Input, T& Output) {
		Output = Input;
		}
		};*/

		template <typename T>
		void Convert(T& Input, T& Output) {
			Output = Input;
		}

		template <typename T, typename K>
		void Convert1(std::vector<T>& Input, vectorBox<K>& Output) {
			Output.clear();
			for (std::size_t i = 0; i < Input.size(); ++i) {
				T& InputElement = Input[i];
				K OutputElement;
				Convert(InputElement, OutputElement);
				Output.push_back(OutputElement);
			}
		}

		template <typename T, typename K>
		void Convert1(vectorBox<T>& Input, std::vector<K>& Output) {
			Output.clear();
			for (std::size_t i = 0; i < Input.size(); ++i) {
				T& InputElement = Input[i];
				K OutputElement;
				Convert(InputElement, OutputElement);
				Output.push_back(OutputElement);
			}
		}

		template <typename T, typename K>
		void Convert2(std::vector<T>& Input, vectorBox<K>& Output) {
			Output.clear();
			for (std::size_t i = 0; i < Input.size(); ++i) {
				T& InputElement = Input[i];
				K OutputElement;
				Convert1(InputElement, OutputElement);
				Output.push_back(OutputElement);
			}
		}

		template <typename T, typename K>
		void Convert2(vectorBox<T>& Input, std::vector<K>& Output) {
			Output.clear();
			for (std::size_t i = 0; i < Input.size(); ++i) {
				T& InputElement = Input[i];
				K OutputElement;
				Convert1(InputElement, OutputElement);
				Output.push_back(OutputElement);
			}
		}

		template <typename T, typename K>
		void Convert3(std::vector<T>& Input, vectorBox<K>& Output) {
			Output.clear();
			for (std::size_t i = 0; i < Input.size(); ++i) {
				T& InputElement = Input[i];
				K OutputElement;
				Convert2(InputElement, OutputElement);
				Output.push_back(OutputElement);
			}
		}

		template <typename T, typename K>
		void Convert3(vectorBox<T>& Input, std::vector<K>& Output) {
			Output.clear();
			for (std::size_t i = 0; i < Input.size(); ++i) {
				T& InputElement = Input[i];
				K OutputElement;
				Convert2(InputElement, OutputElement);
				Output.push_back(OutputElement);
			}
		}

		cv::Point2d GetImagePointFromPointAndBlur(const cv::Point3d& PointAndBlur) {
			return cv::Point2d(PointAndBlur.x, PointAndBlur.y);
		}

		template <int DistortionCoefficientsCount, int CONTROL_POINTS_ORDER>
		bool CeresMultipleCamerasCalibration(const std::vector < std::vector<std::pair<int, std::vector<std::pair<cv::Point3d, cv::Point3d>>>>>& CorrespondencePairsPerCameraPerPatternInput, const std::vector<cv::Size>& CameraResolutions, std::vector<photoneoTools::CameraCalibration64>& OutputCalibrations) {
			//{

			//    std::vector<cv::Point3d> FinalPatternPoints3D;
			//    std::vector<cv::Point3d> FinalReprojectionErrors3D;
			//    FinalPatternPoints3D.push_back(cv::Point3d(0.0, 0.0, 0.0));
			//    FinalPatternPoints3D.push_back(cv::Point3d(0.0, 0.0, 0.0));
			//    FinalPatternPoints3D.push_back(cv::Point3d(0.0, 0.0, 0.0));
			//    FinalPatternPoints3D.push_back(cv::Point3d(0.0, 0.0, 0.0));
			//    FinalPatternPoints3D.push_back(cv::Point3d(0.0, 0.0, 0.0));
			//    FinalPatternPoints3D.push_back(cv::Point3d(0.0, 0.0, 0.0));
			//    FinalPatternPoints3D.push_back(cv::Point3d(0.0, 0.0, 0.0));
			//    FinalPatternPoints3D.push_back(cv::Point3d(0.0, 0.0, 0.0));

			//    FinalReprojectionErrors3D.push_back(cv::Point3d(0.0, 0.0, 0.0));
			//    FinalReprojectionErrors3D.push_back(cv::Point3d(0.0, 0.0, 0.0));
			//    FinalReprojectionErrors3D.push_back(cv::Point3d(0.0, 0.0, 0.0));
			//    FinalReprojectionErrors3D.push_back(cv::Point3d(0.0, 0.0, 0.0));
			//    FinalReprojectionErrors3D.push_back(cv::Point3d(0.0, 0.0, 0.0));
			//    FinalReprojectionErrors3D.push_back(cv::Point3d(0.0, 0.0, 0.0));
			//    FinalReprojectionErrors3D.push_back(cv::Point3d(0.0, 0.0, 0.0));
			//    FinalReprojectionErrors3D.push_back(cv::Point3d(0.0, 0.0, 0.0));

			//    cv::Mat PatternPointsMat(cv::Size(1, FinalPatternPoints3D.size()), CV_32FC3, cv::Scalar(0.0, 0.0, 0.0));
			//    cv::Mat ErrorsMat(cv::Size(1, FinalReprojectionErrors3D.size()), CV_32FC3, cv::Scalar(0.0, 0.0, 0.0));
			//    cv::Mat ReconstructedPointsMat(cv::Size(1, FinalReprojectionErrors3D.size()), CV_32FC3, cv::Scalar(0.0, 0.0, 0.0));

			//    for (std::size_t PointID = 0; PointID < FinalPatternPoints3D.size(); PointID++) {
			//        PatternPointsMat.at<cv::Point3f>(PointID, 0) = (cv::Point3f)FinalPatternPoints3D[PointID];
			//        ErrorsMat.at<cv::Point3f>(PointID, 0) = (cv::Point3f)FinalReprojectionErrors3D[PointID];
			//        ReconstructedPointsMat.at<cv::Point3f>(PointID, 0) = (cv::Point3f)FinalPatternPoints3D[PointID] + (cv::Point3f)FinalReprojectionErrors3D[PointID];
			//    }

			//    /*cv::Mat ErrorsMat(FinalReprojectionErrors3D);
			//    cv::Mat ReconstructedPointsMat = PatternPointsMat + ErrorsMat;

			//    cv::Mat PatternPointsMat(FinalPatternPoints3D);
			//    cv::Mat ErrorsMat(FinalReprojectionErrors3D);
			//    cv::Mat ReconstructedPointsMat = PatternPointsMat + ErrorsMat;*/



			//    //cv::Mat Robot(cv::Size(Sphere.size(), RobotCoordinates.size()), CV_32FC3, cv::Scalar(0.0, 0.0, 0.0));
			//    cv::Mat PatternPointsTexture(PatternPointsMat.size(), CV_8UC3, cv::Scalar(0.0, 255.0, 0.0));
			//    //cv::Mat ICPTransform(cv::Size(Sphere.size(), TransformedICPCoordinates.size()), CV_32FC3, cv::Scalar(0.0, 0.0, 0.0));
			//    cv::Mat ReconstructedPointsTexture(ReconstructedPointsMat.size(), CV_8UC3, cv::Scalar(0.0, 0.0, 255.0));

			//    /*for (std::size_t SphereIndex = 0; SphereIndex < RobotCoordinates.size(); SphereIndex++) {
			//    for (std::size_t PointIndex = 0; PointIndex < Sphere.size(); PointIndex++) {
			//    Robot.at<cv::Point3f>(SphereIndex, PointIndex) = (cv::Point3f)RobotCoordinates[SphereIndex] + Sphere[PointIndex];
			//    ICPTransform.at<cv::Point3f>(SphereIndex, PointIndex) = (cv::Point3f)TransformedICPCoordinates[SphereIndex] + Sphere[PointIndex];
			//    }
			//    }*/

			//    pho::PointCloud PatternPointsCloud(PatternPointsMat, PatternPointsTexture);
			//    pho::PointCloud ReconstructedPointsCloud(ReconstructedPointsMat, ReconstructedPointsTexture);

			//    pho::Plyio::plyWrite("Output/PatternPoints.ply", PatternPointsCloud);
			//    pho::Plyio::plyWrite("Output/ReconstructedPoints.ply", ReconstructedPointsCloud);
			//}



			if (CorrespondencePairsPerCameraPerPatternInput.size() != CameraResolutions.size()) return false;

			std::vector < std::vector<std::pair<int, std::vector<std::pair<cv::Point3d, cv::Point3d>>>>> CorrespondencePairsPerCameraPerPattern = CorrespondencePairsPerCameraPerPatternInput;

			std::set<std::tuple<double, double, double>> ObjectPointSet;


			std::vector<std::vector<std::vector<cv::Point3d>>> ObjectPoints;
			std::vector<std::vector<std::vector<cv::Point2d>>> ImagePoints;
			std::vector<std::vector<std::vector<double>>> PointBlurs;
			std::vector<std::vector<int>> PatternIDs;

			std::vector<std::pair<cv::Vec2i, std::pair<cv::Point2d, cv::Point3d>>> AllCorrespondences;


			std::vector<std::pair<cv::Vec2i, std::pair<cv::Point2d, cv::Point3d>>> FinalReprojectionErrorsAndObjectPoints;

			std::vector<std::vector<std::pair<int, pho::Transformation3D64>>> PatternTransformations;

			std::size_t CamerasCount = CorrespondencePairsPerCameraPerPattern.size();

			OutputCalibrations.resize(CamerasCount);

			std::vector<std::vector<std::unique_ptr<pho::Transformation3D64>>> CameraTransformations(CamerasCount);
			for (std::size_t CameraID = 0; CameraID < CamerasCount; ++CameraID) {
				CameraTransformations[CameraID].resize(CamerasCount);
			}

			int MaxPatternID = 0;

#pragma region InitialCalibrations
			for (std::size_t CameraID = 0; CameraID < CamerasCount; ++CameraID) {
				std::sort(CorrespondencePairsPerCameraPerPattern[CameraID].begin(), CorrespondencePairsPerCameraPerPattern[CameraID].end(), CompareCorrespondencePattern2);
				std::vector<std::vector<cv::Point3d>> ActualObjectPoints;
				std::vector<std::vector<cv::Point2d>> ActualImagePoints;
				std::vector<std::vector<double>> ActualPointBlurs;
				std::vector<int> ActualPatternIDs;
				for (std::size_t PatternID = 0; PatternID < CorrespondencePairsPerCameraPerPattern[CameraID].size(); ++PatternID) {
					std::vector<cv::Point3d> PatternObjectPoints;
					std::vector<cv::Point2d> PatternImagePoints;
					std::vector<double> PatternPointBlurs;
					ActualPatternIDs.push_back(CorrespondencePairsPerCameraPerPattern[CameraID][PatternID].first);
					MaxPatternID = std::max(MaxPatternID, ActualPatternIDs.back());
					std::sort(CorrespondencePairsPerCameraPerPattern[CameraID][PatternID].second.begin(), CorrespondencePairsPerCameraPerPattern[CameraID][PatternID].second.end(), CompareCorrespondencePair2);

					for (std::size_t PointID = 0; PointID < CorrespondencePairsPerCameraPerPattern[CameraID][PatternID].second.size(); ++PointID) {
						PatternObjectPoints.push_back(CorrespondencePairsPerCameraPerPattern[CameraID][PatternID].second[PointID].second);
						ObjectPointSet.insert(std::tuple<double, double, double>(PatternObjectPoints.back().x, PatternObjectPoints.back().y, PatternObjectPoints.back().z));
						PatternImagePoints.push_back(cv::Point2d(CorrespondencePairsPerCameraPerPattern[CameraID][PatternID].second[PointID].first.x, CorrespondencePairsPerCameraPerPattern[CameraID][PatternID].second[PointID].first.y));
						PatternPointBlurs.push_back(CorrespondencePairsPerCameraPerPattern[CameraID][PatternID].second[PointID].first.z);
						AllCorrespondences.push_back(
							std::pair<cv::Vec2i, std::pair<cv::Point2d, cv::Point3d>>(
							cv::Vec2i((int)CameraID, (int)CorrespondencePairsPerCameraPerPattern[CameraID][PatternID].first),
							std::pair<cv::Point2d, cv::Point3d>(GetImagePointFromPointAndBlur(CorrespondencePairsPerCameraPerPattern[CameraID][PatternID].second[PointID].first), CorrespondencePairsPerCameraPerPattern[CameraID][PatternID].second[PointID].second)
							/*CorrespondencePairsPerCameraPerPattern[CameraID][PatternID].second[PointID]*/
							)
							);
						FinalReprojectionErrorsAndObjectPoints.push_back(std::pair<cv::Vec2i, std::pair<cv::Point2d, cv::Point3d>>(cv::Vec2i((int)CameraID, (int)CorrespondencePairsPerCameraPerPattern[CameraID][PatternID].first),
							std::pair<cv::Point2d, cv::Point3d>(GetImagePointFromPointAndBlur(CorrespondencePairsPerCameraPerPattern[CameraID][PatternID].second[PointID].first), CorrespondencePairsPerCameraPerPattern[CameraID][PatternID].second[PointID].second)
							//CorrespondencePairsPerCameraPerPattern[CameraID][PatternID].second[PointID]
							));
					}
					ActualObjectPoints.push_back(PatternObjectPoints);
					ActualImagePoints.push_back(PatternImagePoints);
					ActualPointBlurs.push_back(PatternPointBlurs);
				}
				ObjectPoints.push_back(ActualObjectPoints);
				ImagePoints.push_back(ActualImagePoints);
				PatternIDs.push_back(ActualPatternIDs);
				PointBlurs.push_back(ActualPointBlurs);
			}

			bool Load = false;// true;// false;
			if (Load) {
				vectorBox<vectorBox<vectorBox<cv::Point3d>>> ObjectPointsTemp;
				vectorBox<vectorBox<vectorBox<cv::Point2d>>> ImagePointsTemp;
				vectorBox<vectorBox<vectorBox<double>>> PointBlursTemp;
				vectorBox<vectorBox<int>> PatternIDsTemp;

				vectorBox<std::pair<cv::Vec2i, std::pair<cv::Point2d, cv::Point3d>>> AllCorrespondencesTemp;

				vectorBox<vectorBox<std::pair<int, pho::Transformation3D64>>> PatternTransformationsTemp;

				vectorBox<photoneoTools::CameraCalibration64> OutputCalibrationsTemp;

				cv::FileStorage Storage("Output/CeresMultipleCamerasCalibrationInternmediate.xml", cv::FileStorage::READ);

				Storage["ObjectPoints"] >> ObjectPointsTemp;
				Storage["ImagePoints"] >> ImagePointsTemp;
				Storage["BlurPoints"] >> PointBlursTemp;
				Storage["PatternIDs"] >> PatternIDsTemp;
				Storage["AllCorrespondences"] >> AllCorrespondencesTemp;
				Storage["PatternTransformations"] >> PatternTransformationsTemp;
				Storage["OutputCalibrations"] >> OutputCalibrationsTemp;

				Convert3(ObjectPointsTemp, ObjectPoints);
				Convert3(ImagePointsTemp, ImagePoints);
				Convert3(PointBlursTemp, PointBlurs);
				Convert2(PatternIDsTemp, PatternIDs);
				Convert1(AllCorrespondencesTemp, AllCorrespondences);
				Convert2(PatternTransformationsTemp, PatternTransformations);
				Convert1(OutputCalibrationsTemp, OutputCalibrations);

			}
			else {
				for (std::size_t CameraID = 0; CameraID < CamerasCount; ++CameraID) {

					cv::Mat CameraMatrix = cv::Mat::eye(3, 3, cv::DataType<double>::type);
					if (CameraID == 0) {
						CameraMatrix.at <double>(0, 0) = 2247.0;
						CameraMatrix.at <double>(1, 1) = CameraMatrix.at <double>(0, 0);
					}
					if (CameraID == 1) {
						CameraMatrix.at <double>(0, 0) = 1661.0;
						CameraMatrix.at <double>(1, 1) = CameraMatrix.at <double>(0, 0);
					}
					CameraMatrix.at <double>(0, 2) = ((double)CameraResolutions[CameraID].width / 2.0);
					CameraMatrix.at <double>(1, 2) = ((double)CameraResolutions[CameraID].height / 2.0);
					cv::Mat DistortionCoefficients = cv::Mat::zeros(DistortionCoefficientsCount, 1, cv::DataType<double>::type);


					std::vector<cv::Mat> RVects, TVects;
					std::vector<std::pair<int, pho::Transformation3D64>> ActualPatternTransformations;

					std::vector<std::vector<cv::Point3f>> ActualObjectPoints;
					std::vector<std::vector<cv::Point2f>> ActualImagePoints;
					for (std::size_t i = 0; i < ObjectPoints[CameraID].size(); ++i) {
						std::vector<cv::Point3f> ActualPatternObjectPoints(ObjectPoints[CameraID][i].size());
						std::vector<cv::Point2f> ActualPatternImagePoints(ObjectPoints[CameraID][i].size());
						for (std::size_t k = 0; k < ObjectPoints[CameraID][i].size(); ++k) {
							ActualPatternObjectPoints[k] = (cv::Point3f)ObjectPoints[CameraID][i][k];
							ActualPatternImagePoints[k] = (cv::Point2f)ImagePoints[CameraID][i][k];
						}
						ActualObjectPoints.push_back(ActualPatternObjectPoints);
						ActualImagePoints.push_back(ActualPatternImagePoints);
					}

					double ActualError = cv::calibrateCamera(ActualObjectPoints,
						ActualImagePoints,
						CameraResolutions[CameraID],
						CameraMatrix,
						DistortionCoefficients,
						RVects,
						TVects,
						CV_CALIB_USE_INTRINSIC_GUESS +
						0/*Flags*/ /*+ CV_CALIB_FIX_PRINCIPAL_POINT*/ + (DistortionCoefficientsCount >= 8 ? CV_CALIB_RATIONAL_MODEL
						: 0)
						+ (DistortionCoefficientsCount >= 12
						? CV_CALIB_THIN_PRISM_MODEL : 0)
						+ (DistortionCoefficientsCount >= 14 ? CV_CALIB_TILTED_MODEL
						: 0),
						cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
						5/*0*/,
						2.2204460492503131e-016));

					OutputCalibrations[CameraID].SetCameraMatrix(CameraMatrix);
					OutputCalibrations[CameraID].SetDistortionCoefficients(DistortionCoefficients);
					for (std::size_t PatternID = 0; PatternID < PatternIDs[CameraID].size(); ++PatternID) {
						pho::Transformation3D64 ActualTransformation(pho::RotationVector64(RVects[PatternID].at<double>(0, 0), RVects[PatternID].at<double>(1, 0), RVects[PatternID].at<double>(2, 0)), pho::Translation3D64(TVects[PatternID].at<double>(0, 0), TVects[PatternID].at<double>(1, 0), TVects[PatternID].at<double>(2, 0)));
						ActualPatternTransformations.push_back(std::pair<int, pho::Transformation3D64>(PatternIDs[CameraID][PatternID], ActualTransformation));
					}
					PatternTransformations.push_back(ActualPatternTransformations);
					phoc << OutputCalibrations[CameraID].GetCameraMatrix() << std::endl;
					phoc << OutputCalibrations[CameraID].GetDistortionCoefficients() << std::endl;
				}

				{

					vectorBox<vectorBox<vectorBox<cv::Point3d>>> ObjectPointsTemp;
					vectorBox<vectorBox<vectorBox<cv::Point2d>>> ImagePointsTemp;
					vectorBox<vectorBox<vectorBox<double>>> PointBlursTemp;
					vectorBox<vectorBox<int>> PatternIDsTemp;

					vectorBox<std::pair<cv::Vec2i, std::pair<cv::Point2d, cv::Point3d>>> AllCorrespondencesTemp;

					vectorBox<vectorBox<std::pair<int, pho::Transformation3D64>>> PatternTransformationsTemp;

					vectorBox<photoneoTools::CameraCalibration64> OutputCalibrationsTemp;

					Convert3(ObjectPoints, ObjectPointsTemp);
					Convert3(ImagePoints, ImagePointsTemp);
					Convert3(PointBlurs, PointBlursTemp);
					Convert2(PatternIDs, PatternIDsTemp);
					Convert1(AllCorrespondences, AllCorrespondencesTemp);
					Convert2(PatternTransformations, PatternTransformationsTemp);
					Convert1(OutputCalibrations, OutputCalibrationsTemp);

					cv::FileStorage Storage("Output/CeresMultipleCamerasCalibrationInternmediate.xml", cv::FileStorage::WRITE);
					Storage << "ObjectPoints" << ObjectPointsTemp;
					Storage << "ImagePoints" << ImagePointsTemp;
					Storage << "PointBlurs" << PointBlursTemp;
					Storage << "PatternIDs" << PatternIDsTemp;
					Storage << "AllCorrespondences" << AllCorrespondencesTemp;
					Storage << "PatternTransformations" << PatternTransformationsTemp;
					Storage << "OutputCalibrations" << OutputCalibrationsTemp;
				}
			}
#pragma endregion

#pragma region Get3DTransformations
			for (std::size_t CameraIDFirst = 0; CameraIDFirst < CamerasCount - 1; ++CameraIDFirst) {
				CameraTransformations[CameraIDFirst][CameraIDFirst] = std::unique_ptr<pho::Transformation3D64>(new pho::Transformation3D64());
				for (std::size_t CameraIDSecond = CameraIDFirst + 1; CameraIDSecond < CamerasCount; ++CameraIDSecond) {
					std::vector<std::pair<int, std::vector<std::pair<cv::Point3d, cv::Point3d>>>>& FirstCameraPatternPoints = CorrespondencePairsPerCameraPerPattern[CameraIDFirst];
					std::vector<std::pair<int, std::vector<std::pair<cv::Point3d, cv::Point3d>>>>& SecondCameraPatternPoints = CorrespondencePairsPerCameraPerPattern[CameraIDSecond];

					/*std::vector<std::pair<int, std::vector<std::pair<cv::Point2d, cv::Point3d>>>> FirstCameraPatternPointsFinal;
					std::vector<std::pair<int, std::vector<std::pair<cv::Point2d, cv::Point3d>>>> SecondCameraPatternPointsFinal;*/

					std::vector<cv::Point3d> FirstCameraPatternPointsFinal, SecondCameraPatternPointsFinal;

					std::size_t IDSecond = 0;
					for (std::size_t IDFirst = 0; IDFirst < FirstCameraPatternPoints.size(); ++IDFirst) {
						for (; IDSecond < SecondCameraPatternPoints.size() && FirstCameraPatternPoints[IDFirst].first > SecondCameraPatternPoints[IDSecond].first; ++IDSecond) {
						}
						if (IDSecond < SecondCameraPatternPoints.size() && FirstCameraPatternPoints[IDFirst].first == SecondCameraPatternPoints[IDSecond].first) {
							std::vector<std::pair<cv::Point3d, cv::Point3d>>& FirstPatternPointPairs = FirstCameraPatternPoints[IDFirst].second;
							std::vector<std::pair<cv::Point3d, cv::Point3d>>& SecondPatternPointPairs = SecondCameraPatternPoints[IDSecond].second;
							std::size_t IDSecondPoint = 0;
							for (std::size_t IDFirstPoint = 0; IDFirstPoint < FirstPatternPointPairs.size(); ++IDFirstPoint) {
								for (; IDSecondPoint < SecondPatternPointPairs.size() && CompareCorrespondencePair2(SecondPatternPointPairs[IDSecondPoint], FirstPatternPointPairs[IDFirstPoint]); ++IDSecondPoint) {
								}
								if (IDSecondPoint < SecondPatternPointPairs.size() && SecondPatternPointPairs[IDSecondPoint].second == FirstPatternPointPairs[IDFirstPoint].second) {
									FirstCameraPatternPointsFinal.push_back(PatternTransformations[CameraIDFirst][IDFirst].second.transformPoint(FirstPatternPointPairs[IDFirstPoint].second));
									SecondCameraPatternPointsFinal.push_back(PatternTransformations[CameraIDSecond][IDSecond].second.transformPoint(SecondPatternPointPairs[IDSecondPoint].second));
								}
							}

						}
					}
					if (FirstCameraPatternPointsFinal.size() > 10) {
						pho::Transformation3D64 Transformation = pho::getTransformation3D_mine(FirstCameraPatternPointsFinal, SecondCameraPatternPointsFinal);
						CameraTransformations[CameraIDFirst][CameraIDSecond] = std::unique_ptr<pho::Transformation3D64>(new pho::Transformation3D64(Transformation));
						CameraTransformations[CameraIDSecond][CameraIDFirst] = std::unique_ptr<pho::Transformation3D64>(new pho::Transformation3D64(Transformation.inv()));
					}

				}
			}
#pragma endregion

			//Este treba z kazdej do kazdej
			//Kedze ale mam vstupy zo serea, mozeme sa zatial na to vykalslat ...

			//Make the transformations

			//#define CONTROL_POINTS_ORDER 3

#pragma region InitialSetup
			std::vector<CameraIntrinsicParameters<DistortionCoefficientsCount, CONTROL_POINTS_ORDER, double>> CameraIntrinsic(CamerasCount);
			std::vector<CameraExtendedIntrinsicParameters<DistortionCoefficientsCount, CONTROL_POINTS_ORDER, double>> CameraExtendedIntrinsic(CamerasCount);
			std::vector<cv::Vec3d> PatternTranslationVectors;
			std::vector<cv::Vec4d> PatternRotationQuaternions;
			std::vector<cv::Vec3d> CameraTranslationVectors;
			std::vector<cv::Vec4d> CameraRotationQuaternions;

			for (std::size_t CameraID = 0; CameraID < CamerasCount; ++CameraID) {
				cv::Mat ActualCameraMatrix = OutputCalibrations[CameraID].GetCameraMatrix();
				cv::Mat ActualDistortionCoefficientTemp = OutputCalibrations[CameraID].GetDistortionCoefficients();
				cv::Mat ActualDistortionCoefficient(cv::Size(1, DistortionCoefficientsCount), CV_64FC1, cv::Scalar(0.0));
				if (ActualDistortionCoefficientTemp.rows > DistortionCoefficientsCount) {
					ActualDistortionCoefficientTemp(cv::Rect(0, 0, 1, DistortionCoefficientsCount)).copyTo(ActualDistortionCoefficient);
				}
				else {
					if (ActualDistortionCoefficientTemp.rows < DistortionCoefficientsCount) {
						pho_runtime_error("Incompatible number of Distortion Parameters");
					}
					ActualDistortionCoefficient = ActualDistortionCoefficientTemp;
				}
				OutputCalibrations[CameraID].SetDistortionCoefficients(ActualDistortionCoefficient);
				CameraIntrinsic[CameraID] = CameraIntrinsicParameters<DistortionCoefficientsCount, CONTROL_POINTS_ORDER, double>(OutputCalibrations[CameraID]);
				CameraExtendedIntrinsic[CameraID] = CameraExtendedIntrinsicParameters<DistortionCoefficientsCount, CONTROL_POINTS_ORDER, double>(OutputCalibrations[CameraID]);
			}

			for (std::size_t CameraID = 0; CameraID < CamerasCount; ++CameraID) {
				pho::Transformation3D64 CameraTransformation = *CameraTransformations[0][CameraID];
				pho::Quaternion64 CameraTransformationRotationQuaternion = CameraTransformation.getRotation();

				Vec3d ActualTranslation;
				Vec4d ActualRotation;
				ActualTranslation.val[0] = CameraTransformation.getTranslation().x();
				ActualTranslation.val[1] = CameraTransformation.getTranslation().y();
				ActualTranslation.val[2] = CameraTransformation.getTranslation().z();
				ActualRotation.val[0] = CameraTransformationRotationQuaternion.w();
				ActualRotation.val[1] = CameraTransformationRotationQuaternion.x();
				ActualRotation.val[2] = CameraTransformationRotationQuaternion.y();
				ActualRotation.val[3] = CameraTransformationRotationQuaternion.z();

				CameraTranslationVectors.push_back(ActualTranslation);
				CameraRotationQuaternions.push_back(ActualRotation);
			}

			for (std::size_t PatternID = 0; PatternID < PatternTransformations[0].size(); ++PatternID) {
				pho::Transformation3D64 PatternTransformation = PatternTransformations[0][PatternID].second;
				pho::Quaternion64 PatternTransformationsRotationQuaternion = PatternTransformation.getRotation();
				Vec3d ActualTranslation;
				Vec4d ActualRotation;
				ActualTranslation.val[0] = PatternTransformation.getTranslation().x();
				ActualTranslation.val[1] = PatternTransformation.getTranslation().y();
				ActualTranslation.val[2] = PatternTransformation.getTranslation().z();
				ActualRotation.val[0] = PatternTransformationsRotationQuaternion.w();
				ActualRotation.val[1] = PatternTransformationsRotationQuaternion.x();
				ActualRotation.val[2] = PatternTransformationsRotationQuaternion.y();
				ActualRotation.val[3] = PatternTransformationsRotationQuaternion.z();

				PatternTranslationVectors.push_back(ActualTranslation);
				PatternRotationQuaternions.push_back(ActualRotation);
			}
#pragma endregion

#pragma region InitialCeresOptimization
			phoc("Patterns Count: #", PatternTranslationVectors.size());
			double IterativeTranslation[3] = { 0.0, 0.0, 5.0 };
			if (1){
				ceres::Problem problem;

				for (std::size_t CameraID = 0; CameraID < CameraRotationQuaternions.size(); ++CameraID) {
					problem.AddParameterBlock(CameraRotationQuaternions[CameraID].val, 4, new ceres::QuaternionParameterization());
				}

				for (std::size_t PatternID = 0; PatternID < PatternRotationQuaternions.size(); ++PatternID) {
					problem.AddParameterBlock(PatternRotationQuaternions[PatternID].val, 4, new ceres::QuaternionParameterization());
				}

				for (std::size_t PointID = 0; PointID < AllCorrespondences.size(); ++PointID) {
					cv::Vec2i ID = AllCorrespondences[PointID].first;
					int CameraID = ID.val[0];
					int PatternID = ID.val[1];
					ceres::CostFunction* cost_function = MeasuredCalibrationPointFunctor<DistortionCoefficientsCount>::Create(AllCorrespondences[PointID].second.second, AllCorrespondences[PointID].second.first/*, (double)PatternID*/, FinalReprojectionErrorsAndObjectPoints[PointID].second.second, FinalReprojectionErrorsAndObjectPoints[PointID].second.first);
					problem.AddResidualBlock(cost_function,
						NULL /* squared loss */,
						CameraIntrinsic[CameraID].GetData(),
						PatternTranslationVectors[PatternID].val,
						/*IterativeTranslation,*/
						PatternRotationQuaternions[PatternID].val,
						CameraTranslationVectors[CameraID].val,
						CameraRotationQuaternions[CameraID].val);
				}

				problem.SetParameterBlockConstant(CameraTranslationVectors[0].val);
				problem.SetParameterBlockConstant(CameraRotationQuaternions[0].val);

				ceres::Solver::Options options;
				options.num_threads = 8;
				options.max_num_iterations = 5000;
				//options.sparse_linear_algebra_library_type = ceres::CX_SPARSE;
				//options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
				options.linear_solver_type = ceres::DENSE_SCHUR;
				options.minimizer_progress_to_stdout = true;
				ceres::Solver::Summary summary;
				ceres::Solve(options, &problem, &summary);
				std::cout << summary.FullReport() << std::endl;
				if (!summary.IsSolutionUsable()) {
					return false;// -1.0;
				}
			}
#pragma endregion

			if (1){

				std::vector<cv::Vec<double, DistortionCoefficientsCount>[CONTROL_POINTS_ORDER + 1]> CameraDistortionsControlPoints(CamerasCount);
				std::vector<MicroBezierWarperDistortion<CONTROL_POINTS_ORDER + 1, DistortionCoefficientsCount>> MicroDistortionWarpers;
				std::vector<MicroBezierWarperIntrinsic<CONTROL_POINTS_ORDER + 1, DistortionCoefficientsCount>> MicroIntrinsicWarpers;

				std::vector<std::pair<double, int>> ReprojectionErrorsIndexes;
				for (std::size_t PointID = 0; PointID < FinalReprojectionErrorsAndObjectPoints.size(); ++PointID) {
					ReprojectionErrorsIndexes.push_back(std::pair<double, int>(cv::norm(FinalReprojectionErrorsAndObjectPoints[PointID].second.first), (int)PointID));
				}
				std::sort(ReprojectionErrorsIndexes.begin(), ReprojectionErrorsIndexes.end());
				/*std::ofstream ReprojectionErrorsIndexesStream("Output/ReprojectionErrorsIndexes.txt");

				std::vector<std::pair<cv::Vec2i, std::pair<cv::Point2d, cv::Point3d>>> AllCorrespondences2, AllCorrespondencesOriginal;
				std::vector<std::pair<cv::Vec2i, std::pair<cv::Point2d, cv::Point3d>>> FinalReprojectionErrorsAndObjectPoints2, FinalReprojectionErrorsAndObjectPointsOriginal;


				for (std::size_t i = 0; i < ReprojectionErrorsIndexes.size(); ++i) {
				ReprojectionErrorsIndexesStream << ReprojectionErrorsIndexes[i].first << std::endl;
				}
				for (std::size_t i = 0; i < ReprojectionErrorsIndexes.size() - ReprojectionErrorsIndexes.size() / 3; ++i) {
				AllCorrespondences2.push_back(AllCorrespondences[ReprojectionErrorsIndexes[i].second]);
				FinalReprojectionErrorsAndObjectPoints2.push_back(FinalReprojectionErrorsAndObjectPoints[ReprojectionErrorsIndexes[i].second]);
				}
				ReprojectionErrorsIndexesStream.close();

				AllCorrespondencesOriginal = AllCorrespondences;
				FinalReprojectionErrorsAndObjectPointsOriginal = FinalReprojectionErrorsAndObjectPoints;
				AllCorrespondences = AllCorrespondences2;
				FinalReprojectionErrorsAndObjectPoints = FinalReprojectionErrorsAndObjectPoints2;*/


				for (std::size_t CameraID = 0; CameraID < CamerasCount; ++CameraID) {
					for (int i = 1; i < CONTROL_POINTS_ORDER + 1; i++) {
						CameraIntrinsic[CameraID].SetCameraDistortion(CameraIntrinsic[CameraID].GetCameraDistortion(0), i);
					}
					for (int i = 0; i < CONTROL_POINTS_ORDER + 1; ++i) {
						CameraExtendedIntrinsic[CameraID].SetCameraIntrinsic(CameraIntrinsic[CameraID].GetData(), i);
					}
				}
				std::vector<double> MinDepths(CamerasCount, 0.0);
				std::vector<double> MaxDepths(CamerasCount, 0.0);

				std::vector<cv::Point2d> MaxReprojectionErrors(CamerasCount, cv::Point2d(0.0, 0.0));
				std::vector<int> Points(CamerasCount, 0);
				std::vector<double> AverageReprojectionError(CamerasCount, 0.0);

				for (std::size_t PointID = 0; PointID < FinalReprojectionErrorsAndObjectPoints.size(); ++PointID) {
					cv::Vec2i ID = FinalReprojectionErrorsAndObjectPoints[PointID].first;
					int CameraID = ID.val[0];
					int PatternID = ID.val[1];
					double& MinDepth = MinDepths[CameraID];
					double& MaxDepth = MaxDepths[CameraID];
					cv::Point2d& MaxReprojectionError = MaxReprojectionErrors[CameraID];
					cv::Point3d& ActualPoint = FinalReprojectionErrorsAndObjectPoints[PointID].second.second;
					cv::Point2d& ActualReprojectionError = FinalReprojectionErrorsAndObjectPoints[PointID].second.first;


					++Points[CameraID];
					AverageReprojectionError[CameraID] += cv::norm(ActualReprojectionError);

					if (MinDepth == 0.0) MinDepth = ActualPoint.z;
					if (MaxDepth == 0.0) MaxDepth = ActualPoint.z;
					if (MaxReprojectionError == cv::Point2d(0.0, 0.0)) MaxReprojectionError = ActualReprojectionError;

					if (ActualPoint.z < MinDepth) MinDepth = ActualPoint.z;

					if (ActualPoint.z > MaxDepth) MaxDepth = ActualPoint.z;

					if (ActualReprojectionError.x > MaxReprojectionError.x) MaxReprojectionError.x = ActualReprojectionError.x;
					if (ActualReprojectionError.y > MaxReprojectionError.y) MaxReprojectionError.y = ActualReprojectionError.y;

				}


				std::vector<std::pair<cv::Vec2i, std::pair<cv::Point2d, cv::Point3d>>> AllCorrespondences2, AllCorrespondencesOriginal;
				std::vector<std::pair<cv::Vec2i, std::pair<cv::Point2d, cv::Point3d>>> FinalReprojectionErrorsAndObjectPoints2, FinalReprojectionErrorsAndObjectPointsOriginal;
				{
					std::ofstream ReprojectionErrorsIndexesStream("Output/ReprojectionErrorsIndexes.txt");

					for (std::size_t i = 0; i < ReprojectionErrorsIndexes.size(); ++i) {
						ReprojectionErrorsIndexesStream << ReprojectionErrorsIndexes[i].first << std::endl;
					}
					for (std::size_t i = 0; i < ReprojectionErrorsIndexes.size() /*- ReprojectionErrorsIndexes.size() / 3*/; ++i) {
						AllCorrespondences2.push_back(AllCorrespondences[ReprojectionErrorsIndexes[i].second]);
						FinalReprojectionErrorsAndObjectPoints2.push_back(FinalReprojectionErrorsAndObjectPoints[ReprojectionErrorsIndexes[i].second]);
					}
					ReprojectionErrorsIndexesStream.close();

					AllCorrespondencesOriginal = AllCorrespondences;
					FinalReprojectionErrorsAndObjectPointsOriginal = FinalReprojectionErrorsAndObjectPoints;
					//AllCorrespondences = AllCorrespondences2;
					//FinalReprojectionErrorsAndObjectPoints = FinalReprojectionErrorsAndObjectPoints2;
				}

				double offset = 75.0;

				for (std::size_t CameraID = 0; CameraID < CamerasCount; ++CameraID) {
					if (Points[CameraID]) AverageReprojectionError[CameraID] /= (double)Points[CameraID];

					phoc("Camera #: MinDepth( # ); MaxDepth( # ); MaxReprojectionError( # ); AverageReprojectionError( # )", CameraID, MinDepths[CameraID], MaxDepths[CameraID], MaxReprojectionErrors[CameraID], AverageReprojectionError[CameraID]);

					AverageReprojectionError[CameraID] = 0.0;
					Points[CameraID] = 0;
					MaxReprojectionErrors[CameraID] = cv::Point2d(0.0, 0.0);

					MicroDistortionWarpers.push_back(MicroBezierWarperDistortion<CONTROL_POINTS_ORDER + 1, DistortionCoefficientsCount>(MinDepths[CameraID] - offset, MaxDepths[CameraID] + offset));
					MicroIntrinsicWarpers.push_back(MicroBezierWarperIntrinsic<CONTROL_POINTS_ORDER + 1, DistortionCoefficientsCount>(MinDepths[CameraID] - offset, MaxDepths[CameraID] + offset));

					MinDepths[CameraID] = 0.0;
					MaxDepths[CameraID] = 0.0;
				}


				if (CorrespondencePairsPerCameraPerPattern.size() == 2) {
					ceres::Problem problem;
					bool UseIterativeTranslation = true;// false;// true;
					//double FixedIterativeTranslationLength = 5.0 * 0.99767241379310; //Scale Factor;
					//double FixedIterativeTranslationLength = -1.0;
					double FixedIterativeTranslationLength = 20.0;// 2.5;// 20.0;
					bool OnlyFixTranslationDistance = true;
					bool OnlyFixTranslationDistanceAlternative = false;


					std::vector<int> PatternTranslationReference, PatternIterativeTranslationReference, PatternRotationReference;
					std::vector<double> IterativeTranslationLengths, IterativeTranslationCosts;
					std::vector<cv::Point3d> IterativeTranslationVectors;

					if (!UseIterativeTranslation) {
						PatternTranslationReference.resize(PatternRotationQuaternions.size());
						PatternRotationReference.resize(PatternRotationQuaternions.size());
						PatternIterativeTranslationReference.resize(PatternRotationQuaternions.size());
						IterativeTranslationLengths.resize(PatternRotationQuaternions.size());
						IterativeTranslationVectors.resize(PatternRotationQuaternions.size(), cv::Point3d(0.0, 0.0, 0.0));
						IterativeTranslationCosts.resize(PatternRotationQuaternions.size(), 0.0);
						//IterativeTranslationVectors.resize(CamerasCount);
						//for (std::size_t CameraID = 0; CameraID < CamerasCount; ++CameraID) {
						for (std::size_t PatternID = 0; PatternID < PatternRotationQuaternions.size(); ++PatternID) {
							PatternTranslationReference[PatternID] = PatternID;
							PatternRotationReference[PatternID] = PatternID;
							PatternIterativeTranslationReference[PatternID] = PatternID;
						}
					}
					else {
						PatternTranslationReference.resize(PatternRotationQuaternions.size());
						PatternRotationReference.resize(PatternRotationQuaternions.size());
						PatternIterativeTranslationReference.resize(PatternRotationQuaternions.size());
						IterativeTranslationLengths.resize(PatternRotationQuaternions.size());
						IterativeTranslationVectors.resize(PatternRotationQuaternions.size(), cv::Point3d(0.0, 0.0, 0.0));
						IterativeTranslationCosts.resize(PatternRotationQuaternions.size(), 0.0);
						//IterativeTranslationVectors.resize(CamerasCount);
						//for (std::size_t CameraID = 0; CameraID < CamerasCount; ++CameraID) {
						for (std::size_t PatternID = 0; PatternID < PatternRotationQuaternions.size(); ++PatternID) {
							PatternTranslationReference[PatternID] = 0;
							PatternRotationReference[PatternID] = 0;
							PatternIterativeTranslationReference[PatternID] = 0;
							//IterativeTranslationVectors.push_back();
							if (FixedIterativeTranslationLength > 0.0) {
								IterativeTranslationLengths[PatternID] = (double)PatternID * FixedIterativeTranslationLength;
							}
							else {
								IterativeTranslationLengths[PatternID] = (double)PatternID;
							}
						}
						//}


						if (OnlyFixTranslationDistance) {
							for (std::size_t PatternID = 0; PatternID < PatternRotationQuaternions.size(); ++PatternID) {
								PatternTranslationReference[PatternID] = 0;
								PatternRotationReference[PatternID] = PatternID;
								PatternIterativeTranslationReference[PatternID] = PatternID;

								IterativeTranslationLengths[PatternID] = (double)PatternID * FixedIterativeTranslationLength;
							}
						}
						if (OnlyFixTranslationDistanceAlternative) {
							for (std::size_t PatternID = 0; PatternID < PatternRotationQuaternions.size(); ++PatternID) {
								PatternTranslationReference[PatternID] = 0;
								PatternRotationReference[PatternID] = PatternID;
								PatternIterativeTranslationReference[PatternID] = 0;

								IterativeTranslationLengths[PatternID] = (double)PatternID * FixedIterativeTranslationLength;
							}
						}

						/*for (std::size_t PatternID = 0; PatternID <= 16; ++PatternID) {
						PatternTranslationReference[PatternID] = 0;
						PatternRotationReference[PatternID] = PatternID;
						PatternIterativeTranslationReference[PatternID] = PatternID;

						IterativeTranslationLengths[PatternID] = (double)PatternID * FixedIterativeTranslationLength;
						}
						for (std::size_t PatternID = 17; PatternID <= 17 + 22; ++PatternID) {
						PatternTranslationReference[PatternID] = 17;
						PatternRotationReference[PatternID] = PatternID;
						PatternIterativeTranslationReference[PatternID] = PatternID;

						IterativeTranslationLengths[PatternID] = (double)(PatternID - 17) * FixedIterativeTranslationLength;
						}*/


						for (std::size_t PatternID = 0; PatternID < PatternRotationQuaternions.size(); ++PatternID) {
							if (IterativeTranslationLengths[PatternID] > 0.0) {
								cv::Point3d TranslationDifference = (cv::Point3d)(
									PatternTranslationVectors[PatternID] -
									PatternTranslationVectors[PatternTranslationReference[PatternID]]
									);
								TranslationDifference /= IterativeTranslationLengths[PatternID];
								IterativeTranslationCosts[PatternIterativeTranslationReference[PatternID]] += 1.0;

								{
									pho::Quaternion64 quaternion(PatternRotationQuaternions[PatternRotationReference[PatternID]].val[0], PatternRotationQuaternions[PatternRotationReference[PatternID]].val[1], PatternRotationQuaternions[PatternRotationReference[PatternID]].val[2], PatternRotationQuaternions[PatternRotationReference[PatternID]].val[3]);
									pho::Rotation3D64 Rot(quaternion);

									cv::Point3d Translation = PatternTranslationVectors[PatternTranslationReference[PatternID]];
									pho::Translation3D64 Trans(Translation.x, Translation.y, Translation.z);
									pho::Transformation3D64 Transformation(Rot, Trans);
									IterativeTranslationVectors[PatternIterativeTranslationReference[PatternID]] += Transformation.inv().transformVector(TranslationDifference);
								}

								//IterativeTranslationVectors[PatternIterativeTranslationReference[PatternID]] += TranslationDifference;
							}
						}
						for (std::size_t PatternID = 0; PatternID < PatternRotationQuaternions.size(); ++PatternID) {
							if (IterativeTranslationCosts[PatternID] > 0.0) {
								IterativeTranslationVectors[PatternID] *= 1.0 / (IterativeTranslationCosts[PatternID]);

								if (FixedIterativeTranslationLength > 0.0) {
									double Length = cv::norm(IterativeTranslationVectors[PatternID]);
									IterativeTranslationVectors[PatternID] *= 1.0 / Length;

									//phoc("IterativeTranslationVector #: #", PatternID, IterativeTranslationVectors[PatternID]);
									if (IterativeTranslationVectors[PatternID].z * ITERATIVE_TRANSLATION_VECTOR_MULTIPLIER < 0.0) {
										pho_runtime_error("Axis of the pattern iterative translation direction is incorrect, check ITERATIVE_TRANSLATION_VECTOR_MULTIPLIER");
									}

								}
							}


						}

						//Just for test
						/*for (std::size_t PatternID = 1; PatternID <= PatternRotationQuaternions.size(); ++PatternID) {
						PatternTranslationReference[PatternID] = PatternID;
						PatternTranslationVectors[PatternID] = PatternTranslationVectors[0];
						}*/
					}




					for (std::size_t CameraID = 1; CameraID < CameraRotationQuaternions.size(); ++CameraID) {
						problem.AddParameterBlock(CameraRotationQuaternions[CameraID].val, 4, new ceres::QuaternionParameterization());
					}

					for (std::size_t CameraID = 0; CameraID < CamerasCount; ++CameraID) {
						//problem.AddParameterBlock(CameraIntrinsic[CameraID].GetData(), 2 + 2 + DistortionCoefficientsCount * (CONTROL_POINTS_ORDER + 1));
						problem.AddParameterBlock(CameraExtendedIntrinsic[CameraID].GetData(), (2 + 2 + DistortionCoefficientsCount) * (CONTROL_POINTS_ORDER + 1));
					}

					for (std::size_t PatternID = 0; PatternID < PatternRotationQuaternions.size(); ++PatternID) {
						problem.AddParameterBlock(PatternRotationQuaternions[PatternID].val, 4, new ceres::QuaternionParameterization());
					}
					if (!UseIterativeTranslation) {
						/*for (std::size_t PatternID = 0; PatternID < PatternRotationQuaternions.size(); ++PatternID) {
						problem.AddParameterBlock(PatternRotationQuaternions[PatternID].val, 4, new ceres::QuaternionParameterization());
						}*/
					}
					else {
						IterativeTranslation[0] = 0.0;
						IterativeTranslation[1] = 0.0;
						IterativeTranslation[2] = 0.0;
						for (std::size_t PatternID = 1; PatternID < PatternTranslationVectors.size(); ++PatternID) {
							IterativeTranslation[0] += (PatternTranslationVectors[PatternID] - PatternTranslationVectors[PatternID - 1]).val[0];
							IterativeTranslation[1] += (PatternTranslationVectors[PatternID] - PatternTranslationVectors[PatternID - 1]).val[1];
							IterativeTranslation[2] += (PatternTranslationVectors[PatternID] - PatternTranslationVectors[PatternID - 1]).val[2];
						}
						IterativeTranslation[0] /= (double)(PatternTranslationVectors.size() - 1);
						IterativeTranslation[1] /= (double)(PatternTranslationVectors.size() - 1);
						IterativeTranslation[2] /= (double)(PatternTranslationVectors.size() - 1);

						if (FixedIterativeTranslationLength > 0.0) {
							double Length = std::sqrt(
								IterativeTranslation[0] * IterativeTranslation[0] +
								IterativeTranslation[1] * IterativeTranslation[1] +
								IterativeTranslation[2] * IterativeTranslation[2]
								);
							IterativeTranslation[0] /= Length;
							IterativeTranslation[1] /= Length;
							IterativeTranslation[2] /= Length;
						}

						//problem.AddParameterBlock(PatternRotationQuaternions[0].val, 4, new ceres::QuaternionParameterization());
					}

					cv::Point3d TempPoint3;
					cv::Point2d TempPoint2;

					/*std::vector<cv::Point3d> FinalReprojectionErrors3D(AllCorrespondences.size());
					std::vector<cv::Point3d> FinalPatternPoints3D(AllCorrespondences.size());*/

					std::vector<cv::Point3d> FinalReprojectionErrors3D;
					std::vector<cv::Point3d> FinalPatternPoints3D;
					std::vector<double> FinalPointBlurs[2];
					double MaxBlurs[2] = { 0.0, 0.0 };
					FinalReprojectionErrors3D.reserve(AllCorrespondences.size());
					FinalPatternPoints3D.reserve(AllCorrespondences.size());

					std::vector<cv::Point3d> NearestPointLineVectors3D;
					NearestPointLineVectors3D.reserve(AllCorrespondences.size());

					std::vector<std::vector<cv::Point2d>> ImagePoints(CamerasCount);
					std::vector<int> PatternIDs;

					std::size_t Index = 0;
					std::size_t SecondPatternID = 0;

					for (std::size_t FirstPatternID = 0; FirstPatternID < CorrespondencePairsPerCameraPerPattern[0].size(); ++FirstPatternID) {
						int PatternID = CorrespondencePairsPerCameraPerPattern[0][FirstPatternID].first;
						while (SecondPatternID < CorrespondencePairsPerCameraPerPattern[1].size() && CorrespondencePairsPerCameraPerPattern[1][SecondPatternID].first < PatternID) ++SecondPatternID;
						if (SecondPatternID < CorrespondencePairsPerCameraPerPattern[1].size() && CorrespondencePairsPerCameraPerPattern[1][SecondPatternID].first == PatternID) {
							std::vector<std::pair<cv::Point3d, cv::Point3d>>& FirstPatternPoints = CorrespondencePairsPerCameraPerPattern[0][FirstPatternID].second;
							std::vector<std::pair<cv::Point3d, cv::Point3d>>& SecondPatternPoints = CorrespondencePairsPerCameraPerPattern[1][SecondPatternID].second;

							std::size_t SecondPointID = 0;
							for (std::size_t FirstPointID = 0; FirstPointID < FirstPatternPoints.size(); ++FirstPointID) {
								while (SecondPointID < SecondPatternPoints.size() && CompareCorrespondencePair2(SecondPatternPoints[SecondPointID], FirstPatternPoints[FirstPointID])) ++SecondPointID;
								if (SecondPointID < SecondPatternPoints.size() && SecondPatternPoints[SecondPointID].second == FirstPatternPoints[FirstPointID].second) {
									FinalReprojectionErrors3D.push_back(cv::Point3d(0.0, 0.0, 0.0));
									FinalPatternPoints3D.push_back(cv::Point3d(0.0, 0.0, 0.0));
									NearestPointLineVectors3D.push_back(cv::Point3d(0.0, 0.0, 0.0));
									FinalPointBlurs[0].push_back(FirstPatternPoints[FirstPointID].first.z);
									FinalPointBlurs[1].push_back(SecondPatternPoints[SecondPointID].first.z);
									MaxBlurs[0] = std::max(MaxBlurs[0], FinalPointBlurs[0].back());
									MaxBlurs[1] = std::max(MaxBlurs[1], FinalPointBlurs[1].back());

									if (UseIterativeTranslation) {
										//MeasuredCalibrationPointWithDistortionWarpReversedFunctor<DistortionCoefficientsCount, CONTROL_POINTS_ORDER> ActualFunctor(FirstPatternPoints[FirstPointID].second, FirstPatternPoints[FirstPointID].first, SecondPatternPoints[SecondPointID].first, FinalPatternPoints3D.back(), FinalReprojectionErrors3D.back(), MicroDistortionWarpers[0], MicroDistortionWarpers[1], (FixedIterativeTranslationLength > 0.0) ? FixedIterativeTranslationLength * double(PatternID) : double(PatternID), FixedIterativeTranslationLength > 0.0);
										MeasuredCalibrationPointWithIntrinsicWarpReversedFunctor<DistortionCoefficientsCount, CONTROL_POINTS_ORDER>
											ActualFunctor(FirstPatternPoints[FirstPointID].second, GetImagePointFromPointAndBlur(FirstPatternPoints[FirstPointID].first), GetImagePointFromPointAndBlur(SecondPatternPoints[SecondPointID].first), FinalPatternPoints3D.back(), FinalReprojectionErrors3D.back(), NearestPointLineVectors3D.back(), MicroIntrinsicWarpers[0], MicroIntrinsicWarpers[1],
											//(FixedIterativeTranslationLength > 0.0) ? FixedIterativeTranslationLength * double(PatternID) : double(PatternID),
											IterativeTranslationLengths[PatternID],
											FixedIterativeTranslationLength > 0.0);
										double Residual[3];
										ActualFunctor(
											/*CameraIntrinsic[0].GetData(),
											CameraIntrinsic[1].GetData(),*/
											CameraExtendedIntrinsic[0].GetData(),
											CameraExtendedIntrinsic[1].GetData(),
											PatternTranslationVectors[PatternTranslationReference[PatternID]].val,
											&IterativeTranslationVectors[PatternIterativeTranslationReference[PatternID]].x,
											//IterativeTranslation,
											PatternRotationQuaternions[PatternRotationReference[PatternID]].val,
											CameraTranslationVectors[1].val,
											CameraRotationQuaternions[1].val,
											Residual);
									}
									else {
										//MeasuredCalibrationPointWithDistortionWarpReversedFunctor<DistortionCoefficientsCount, CONTROL_POINTS_ORDER> ActualFunctor(FirstPatternPoints[FirstPointID].second, FirstPatternPoints[FirstPointID].first, SecondPatternPoints[SecondPointID].first, FinalPatternPoints3D.back(), FinalReprojectionErrors3D.back(), MicroDistortionWarpers[0], MicroDistortionWarpers[1]);
										MeasuredCalibrationPointWithIntrinsicWarpReversedFunctor<DistortionCoefficientsCount, CONTROL_POINTS_ORDER> ActualFunctor(FirstPatternPoints[FirstPointID].second, GetImagePointFromPointAndBlur(FirstPatternPoints[FirstPointID].first), GetImagePointFromPointAndBlur(SecondPatternPoints[SecondPointID].first), FinalPatternPoints3D.back(), FinalReprojectionErrors3D.back(), NearestPointLineVectors3D.back(), MicroIntrinsicWarpers[0], MicroIntrinsicWarpers[1]);
										double Residual[3];
										ActualFunctor(
											/*CameraIntrinsic[0].GetData(),
											CameraIntrinsic[1].GetData(),*/
											CameraExtendedIntrinsic[0].GetData(),
											CameraExtendedIntrinsic[1].GetData(),
											PatternTranslationVectors[PatternID].val,
											PatternRotationQuaternions[PatternID].val,
											CameraTranslationVectors[1].val,
											CameraRotationQuaternions[1].val,
											Residual);
									}

									ImagePoints[0].push_back(GetImagePointFromPointAndBlur(FirstPatternPoints[FirstPointID].first));
									ImagePoints[1].push_back(GetImagePointFromPointAndBlur(SecondPatternPoints[SecondPointID].first));
									PatternIDs.push_back((int)PatternID);
									ceres::CostFunction* cost_function;
									if (UseIterativeTranslation) {
										if (FixedIterativeTranslationLength > 0.0) {
											//cost_function = MeasuredCalibrationPointWithDistortionWarpReversedFunctor<DistortionCoefficientsCount, CONTROL_POINTS_ORDER>::CreateBoundedTranslationOrientation(FirstPatternPoints[FirstPointID].second, FirstPatternPoints[FirstPointID].first, SecondPatternPoints[SecondPointID].first, FinalPatternPoints3D.back(), FinalReprojectionErrors3D.back(), MicroDistortionWarpers[0], MicroDistortionWarpers[1], (FixedIterativeTranslationLength > 0.0) ? FixedIterativeTranslationLength * double(PatternID) : double(PatternID));
											cost_function = MeasuredCalibrationPointWithIntrinsicWarpReversedFunctor<DistortionCoefficientsCount, CONTROL_POINTS_ORDER>::CreateBoundedTranslationOrientation(FirstPatternPoints[FirstPointID].second, GetImagePointFromPointAndBlur(FirstPatternPoints[FirstPointID].first), GetImagePointFromPointAndBlur(SecondPatternPoints[SecondPointID].first), FinalPatternPoints3D.back(), FinalReprojectionErrors3D.back(), NearestPointLineVectors3D.back(), MicroIntrinsicWarpers[0], MicroIntrinsicWarpers[1],
												//(FixedIterativeTranslationLength > 0.0) ? FixedIterativeTranslationLength * double(PatternID) : double(PatternID)
												IterativeTranslationLengths[PatternID]
												);
										}
										else {
											//cost_function = MeasuredCalibrationPointWithDistortionWarpReversedFunctor<DistortionCoefficientsCount, CONTROL_POINTS_ORDER>::CreateBoundedTranslation(FirstPatternPoints[FirstPointID].second, FirstPatternPoints[FirstPointID].first, SecondPatternPoints[SecondPointID].first, FinalPatternPoints3D.back(), FinalReprojectionErrors3D.back(), MicroDistortionWarpers[0], MicroDistortionWarpers[1], double(PatternID));
											cost_function = MeasuredCalibrationPointWithIntrinsicWarpReversedFunctor<DistortionCoefficientsCount, CONTROL_POINTS_ORDER>::CreateBoundedTranslation(FirstPatternPoints[FirstPointID].second, GetImagePointFromPointAndBlur(FirstPatternPoints[FirstPointID].first), GetImagePointFromPointAndBlur(SecondPatternPoints[SecondPointID].first), FinalPatternPoints3D.back(), FinalReprojectionErrors3D.back(), NearestPointLineVectors3D.back(), MicroIntrinsicWarpers[0], MicroIntrinsicWarpers[1],
												//double(PatternID)
												IterativeTranslationLengths[PatternID]
												);
										}
										problem.AddResidualBlock(cost_function,
											NULL /* squared loss */,
											/*CameraIntrinsic[0].GetData(),
											CameraIntrinsic[1].GetData(),*/
											CameraExtendedIntrinsic[0].GetData(),
											CameraExtendedIntrinsic[1].GetData(),
											PatternTranslationVectors[PatternTranslationReference[PatternID]].val,
											&IterativeTranslationVectors[PatternIterativeTranslationReference[PatternID]].x,
											//IterativeTranslation,
											PatternRotationQuaternions[PatternRotationReference[PatternID]].val,
											CameraTranslationVectors[1].val,
											CameraRotationQuaternions[1].val);
									}
									else {
										//cost_function = MeasuredCalibrationPointWithDistortionWarpReversedFunctor<DistortionCoefficientsCount, CONTROL_POINTS_ORDER>::Create(FirstPatternPoints[FirstPointID].second, FirstPatternPoints[FirstPointID].first, SecondPatternPoints[SecondPointID].first, FinalPatternPoints3D.back(), FinalReprojectionErrors3D.back(), MicroDistortionWarpers[0], MicroDistortionWarpers[1]);
										cost_function = MeasuredCalibrationPointWithIntrinsicWarpReversedFunctor<DistortionCoefficientsCount, CONTROL_POINTS_ORDER>::Create(FirstPatternPoints[FirstPointID].second, GetImagePointFromPointAndBlur(FirstPatternPoints[FirstPointID].first), GetImagePointFromPointAndBlur(SecondPatternPoints[SecondPointID].first), FinalPatternPoints3D.back(), FinalReprojectionErrors3D.back(), NearestPointLineVectors3D.back(), MicroIntrinsicWarpers[0], MicroIntrinsicWarpers[1]);
										problem.AddResidualBlock(cost_function,
											NULL /* squared loss */,
											/*CameraIntrinsic[0].GetData(),
											CameraIntrinsic[1].GetData(),*/
											CameraExtendedIntrinsic[0].GetData(),
											CameraExtendedIntrinsic[1].GetData(),
											PatternTranslationVectors[PatternID].val,
											PatternRotationQuaternions[PatternID].val,
											CameraTranslationVectors[1].val,
											CameraRotationQuaternions[1].val);
									}
									++Index;
									++SecondPointID;
								}
							}
							++SecondPatternID;
						}
					}

					/*problem.SetParameterBlockConstant(CameraTranslationVectors[0].val);
					problem.SetParameterBlockConstant(CameraRotationQuaternions[0].val);*/

					{
						double AverageError = 0.0;
						std::vector<double> FinalReprojectionErrors3DNorm;
						double MaxError = cv::norm(FinalReprojectionErrors3D[0]);
						for (std::size_t i = 0; i < FinalReprojectionErrors3D.size(); ++i) {
							FinalReprojectionErrors3DNorm.push_back(cv::norm(FinalReprojectionErrors3D[i]));
							MaxError = std::max(MaxError, FinalReprojectionErrors3DNorm.back());
							AverageError += FinalReprojectionErrors3DNorm.back();
						}
						AverageError *= 1.0 / (double)FinalReprojectionErrors3D.size();

						double StandartDeviation = 0;
						for (std::size_t i = 0; i < FinalReprojectionErrors3DNorm.size(); ++i) {
							StandartDeviation += std::pow(FinalReprojectionErrors3DNorm[i] - AverageError, 2.0);
						}
						StandartDeviation /= (double)FinalReprojectionErrors3D.size();
						StandartDeviation = std::sqrt(StandartDeviation);


						std::sort(FinalReprojectionErrors3DNorm.begin(), FinalReprojectionErrors3DNorm.end());
						std::ofstream ReprojectionErrorsIndexesStream("Output/ReprojectionErrorsIndexesInitial3D.txt");
						std::ofstream NearestPointLineVectors3DStream("Output/NearestPointLineVectors3DInitial.txt");
						for (std::size_t i = 0; i < FinalReprojectionErrors3DNorm.size(); ++i) {
							ReprojectionErrorsIndexesStream << FinalReprojectionErrors3DNorm[i] << std::endl;
							NearestPointLineVectors3DStream << NearestPointLineVectors3D[i] << std::endl;
						}
						ReprojectionErrorsIndexesStream.close();
						NearestPointLineVectors3DStream.close();

						if (FixedIterativeTranslationLength > 0.0) {
							IterativeTranslation[2] = ITERATIVE_TRANSLATION_VECTOR_MULTIPLIER * std::sqrt(1.0 - IterativeTranslation[0] * IterativeTranslation[0] - IterativeTranslation[1] * IterativeTranslation[1]);
						}

						phoc("Initial Camera 3D Test: MaxReprojectionError( # ); AverageReprojectionError( # ); STD( # ); TranslationVector (#; #; #)", MaxError, AverageError, StandartDeviation, IterativeTranslation[0], IterativeTranslation[1], IterativeTranslation[2]);

						for (std::size_t CameraID = 0; CameraID < CamerasCount; ++CameraID) {
							/*phoc("Camera # params: FOV: [# ; #], Principal Point: [# ; #], Distortion Coefficients:", CameraID, CameraIntrinsic[CameraID].GetCameraFOV()[0], CameraIntrinsic[CameraID].GetCameraFOV()[1], CameraIntrinsic[CameraID].GetCameraPrincipalPoint()[0], CameraIntrinsic[CameraID].GetCameraPrincipalPoint()[1]);
							for (int Order = 0; Order < CONTROL_POINTS_ORDER + 1; ++Order) {
							for (int i = 0; i < DistortionCoefficientsCount; ++i) {
							phoc << CameraIntrinsic[CameraID].GetCameraDistortion(Order)[i] << ";  ";
							}
							phoc << std::endl;
							}*/
							for (int Order = 0; Order < CONTROL_POINTS_ORDER + 1; ++Order) {
								phoc("Camera # params: FOV: [# ; #], Principal Point: [# ; #], Distortion Coefficients:", CameraID, CameraExtendedIntrinsic[CameraID].GetCameraFOV(Order)[0], CameraExtendedIntrinsic[CameraID].GetCameraFOV(Order)[1], CameraExtendedIntrinsic[CameraID].GetCameraPrincipalPoint(Order)[0], CameraExtendedIntrinsic[CameraID].GetCameraPrincipalPoint(Order)[1]);
								for (int i = 0; i < DistortionCoefficientsCount; ++i) {
									phoc << CameraExtendedIntrinsic[CameraID].GetCameraDistortion(Order)[i] << ";  ";
								}
								phoc << std::endl;
							}

						}
					}

					ceres::Solver::Options options;
					options.num_threads = 8;
					//options.sparse_linear_algebra_library_type = ceres::CX_SPARSE;
					//options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
					options.linear_solver_type = ceres::DENSE_SCHUR;
					//options.linear_solver_ordering = ceres::SCHUR;
					options.max_num_iterations = 5000;
					options.minimizer_progress_to_stdout = true;
					ceres::Solver::Summary summary;
					ceres::Solve(options, &problem, &summary);
					std::cout << summary.BriefReport() << std::endl;




					//if (!UseIterativeTranslation) {
					std::ofstream ReprojectionErrorsIndexesFinal3DRotations("Output/ReprojectionErrorsIndexesFinal3DRotations.txt");
					std::ofstream ReprojectionErrorsIndexesFinal3DTranslations("Output/ReprojectionErrorsIndexesFinal3DTranslations.txt");
					std::ofstream ReprojectionErrorsIndexesFinal3DTranslationsIterative("Output/ReprojectionErrorsIndexesFinal3DTranslationsIterative.txt");
					for (std::size_t PatternID = 0; PatternID < PatternRotationQuaternions.size(); ++PatternID) {
						pho::Quaternion64 quaternion(PatternRotationQuaternions[PatternRotationReference[PatternID]].val[0], PatternRotationQuaternions[PatternRotationReference[PatternID]].val[1], PatternRotationQuaternions[PatternRotationReference[PatternID]].val[2], PatternRotationQuaternions[PatternRotationReference[PatternID]].val[3]);
						pho::Rotation3D64 Rot(quaternion);
						pho::EulerAngles64 Angles(Rot);
						ReprojectionErrorsIndexesFinal3DRotations << phoc.ToString("#; #; #", Angles.a(), Angles.b(), Angles.c()) << std::endl;
						//ReprojectionErrorsIndexesFinal3DRotations << PatternRotationQuaternions[PatternID] << std::endl;
					}
					cv::Point3d CurrentTranslation, LastTranslation;
					for (std::size_t PatternID = 0; PatternID < PatternTranslationVectors.size(); ++PatternID) {
						if (PatternTranslationReference.size() > PatternID) {
							if (FixedIterativeTranslationLength > 0.0) {
								IterativeTranslationVectors[PatternID];
								IterativeTranslationVectors[PatternID].z = ITERATIVE_TRANSLATION_VECTOR_MULTIPLIER * std::sqrt(1.0 - IterativeTranslationVectors[PatternID].x * IterativeTranslationVectors[PatternID].x - IterativeTranslationVectors[PatternID].y * IterativeTranslationVectors[PatternID].y);
							}
						}
					}
					for (std::size_t PatternID = 0; PatternID < PatternTranslationVectors.size(); ++PatternID) {
						ReprojectionErrorsIndexesFinal3DTranslations << PatternTranslationVectors[PatternTranslationReference[PatternID]];
						if (PatternID > 0) ReprojectionErrorsIndexesFinal3DTranslations << " " << PatternTranslationVectors[PatternTranslationReference[PatternID]] - PatternTranslationVectors[PatternTranslationReference[PatternID - 1]] << " " << cv::norm(PatternTranslationVectors[PatternID] - PatternTranslationVectors[PatternID - 1]) << " " << cv::norm(PatternTranslationVectors[PatternTranslationReference[PatternID]] - PatternTranslationVectors[PatternTranslationReference[0]]);
						ReprojectionErrorsIndexesFinal3DTranslations << std::endl;

						if (PatternTranslationReference.size() > PatternID) {
							CurrentTranslation = (cv::Point3d)PatternTranslationVectors[PatternTranslationReference[PatternID]] + (cv::Point3d)IterativeTranslationVectors[PatternIterativeTranslationReference[PatternID]] * IterativeTranslationLengths[PatternID];
							ReprojectionErrorsIndexesFinal3DTranslationsIterative << CurrentTranslation;
							if (PatternID > 0) {
								ReprojectionErrorsIndexesFinal3DTranslationsIterative << " " << CurrentTranslation - LastTranslation << " " << cv::norm(CurrentTranslation - LastTranslation);
							}
							ReprojectionErrorsIndexesFinal3DTranslationsIterative << std::endl;
							LastTranslation = CurrentTranslation;
						}
					}
					ReprojectionErrorsIndexesFinal3DRotations.close();
					ReprojectionErrorsIndexesFinal3DTranslations.close();
					ReprojectionErrorsIndexesFinal3DTranslationsIterative.close();


					std::vector<pho::Transformation3D64> FinalPatternTransformations;
					for (std::size_t PatternID = 0; PatternID < PatternRotationQuaternions.size(); ++PatternID) {
						pho::Quaternion64 quaternion(PatternRotationQuaternions[PatternRotationReference[PatternID]].val[0], PatternRotationQuaternions[PatternRotationReference[PatternID]].val[1], PatternRotationQuaternions[PatternRotationReference[PatternID]].val[2], PatternRotationQuaternions[PatternRotationReference[PatternID]].val[3]);
						pho::Rotation3D64 Rot(quaternion);

						cv::Point3d TranslationAfter = PatternTranslationVectors[PatternTranslationReference[PatternID]];
						if (UseIterativeTranslation);
						cv::Point3d TranslationBefore = IterativeTranslationVectors[PatternIterativeTranslationReference[PatternID]] * IterativeTranslationLengths[PatternID];

						pho::Translation3D64 TransAfter(TranslationAfter.x, TranslationAfter.y, TranslationAfter.z);
						pho::Translation3D64 TransBefore(TranslationBefore.x, TranslationBefore.y, TranslationBefore.z);
						FinalPatternTransformations.push_back(pho::Transformation3D64(TransBefore) * pho::Transformation3D64(Rot, TransAfter));
					}
					//}


					{
						std::vector<cv::Mat> DebugImages(CamerasCount);
						std::vector<std::vector<cv::Mat>> PerPatternDebugImage(CamerasCount);

						for (std::size_t CameraID = 0; CameraID < CamerasCount; ++CameraID) {
							DebugImages[CameraID] = cv::Mat(CameraResolutions[CameraID], CV_8UC3, cv::Scalar(0.0, 0.0, 0.0));
						}

						double AverageError = 0.0;
						std::vector<std::pair<double, int>> FinalReprojectionErrors3DNorm;
						double MaxError = cv::norm(FinalReprojectionErrors3D[0]);
						for (std::size_t i = 0; i < FinalReprojectionErrors3D.size(); ++i) {
							FinalReprojectionErrors3DNorm.push_back(std::pair<double, int>(cv::norm(FinalReprojectionErrors3D[i]), i));
							MaxError = std::max(MaxError, FinalReprojectionErrors3DNorm.back().first);
							AverageError += FinalReprojectionErrors3DNorm.back().first;
						}
						AverageError *= 1.0 / (double)FinalReprojectionErrors3D.size();

						std::sort(FinalReprojectionErrors3DNorm.begin(), FinalReprojectionErrors3DNorm.end());
						std::ofstream ReprojectionErrorsIndexesStream("Output/ReprojectionErrorsIndexesFinal3D.txt");
						std::ofstream NearestPointLineVectors3DStream("Output/NearestPointLineVectors3DFinal.txt");


						double StandartDeviation = 0.0;
						for (std::size_t i = 0; i < FinalReprojectionErrors3DNorm.size(); ++i) {
							double ActualError = FinalReprojectionErrors3DNorm[i].first;
							StandartDeviation += std::pow(ActualError - AverageError, 2.0);
							ActualError /= MaxError;
							cv::circle(DebugImages[0], ImagePoints[0][FinalReprojectionErrors3DNorm[i].second], 10, cv::Scalar(255.0 * ActualError, 255.0 * ActualError, 255.0 * ActualError), -1);
							cv::circle(DebugImages[1], ImagePoints[1][FinalReprojectionErrors3DNorm[i].second], 10, cv::Scalar(255.0 * ActualError, 255.0 * ActualError, 255.0 * ActualError), -1);

							for (std::size_t CameraID = 0; CameraID < CamerasCount; ++CameraID) {
								int PatternID = PatternIDs[FinalReprojectionErrors3DNorm[i].second];
								if (PatternID >= (int)PerPatternDebugImage[CameraID].size()) {
									PerPatternDebugImage[CameraID].resize(PatternID + 1);
								}
								if (PerPatternDebugImage[CameraID][PatternID].empty()) {
									PerPatternDebugImage[CameraID][PatternID] = cv::Mat(CameraResolutions[CameraID], CV_32FC1, cv::Scalar(0.0, 0.0, 0.0));
								}
								cv::circle(PerPatternDebugImage[CameraID][PatternID], ImagePoints[CameraID][FinalReprojectionErrors3DNorm[i].second], 10, cv::Scalar(FinalReprojectionErrors3DNorm[i].first), -1);
								cv::putText(PerPatternDebugImage[CameraID][PatternID], std::to_string(FinalPointBlurs[CameraID][FinalReprojectionErrors3DNorm[i].second]).substr(0, 5), ImagePoints[CameraID][FinalReprojectionErrors3DNorm[i].second] + cv::Point2d(10.0, 10.0), cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(1.0));
							}

							ReprojectionErrorsIndexesStream << FinalReprojectionErrors3DNorm[i].first << std::endl;
							NearestPointLineVectors3DStream << NearestPointLineVectors3D[i] << std::endl;
						}
						ReprojectionErrorsIndexesStream.close();
						NearestPointLineVectors3DStream.close();
						StandartDeviation /= (double)FinalReprojectionErrors3DNorm.size();
						StandartDeviation = std::sqrt(StandartDeviation);

						if (FixedIterativeTranslationLength > 0.0) {
							IterativeTranslation[2] = ITERATIVE_TRANSLATION_VECTOR_MULTIPLIER * std::sqrt(1.0 - IterativeTranslation[0] * IterativeTranslation[0] - IterativeTranslation[1] * IterativeTranslation[1]);
						}

						phoc("Final Camera 3D Test: MaxReprojectionError( # ); AverageReprojectionError( # ); STD( # ); TranslationVector (#; #; #)", MaxError, AverageError, StandartDeviation, IterativeTranslation[0], IterativeTranslation[1], IterativeTranslation[2]);

						for (std::size_t CameraID = 0; CameraID < CamerasCount; ++CameraID) {
							/*phoc("Camera # params: FOV: [# ; #], Principal Point: [# ; #], Distortion Coefficients:", CameraID, CameraIntrinsic[CameraID].GetCameraFOV()[0], CameraIntrinsic[CameraID].GetCameraFOV()[1], CameraIntrinsic[CameraID].GetCameraPrincipalPoint()[0], CameraIntrinsic[CameraID].GetCameraPrincipalPoint()[1]);
							for (int Order = 0; Order < CONTROL_POINTS_ORDER + 1; ++Order) {
							for (int i = 0; i < DistortionCoefficientsCount; ++i) {
							phoc << CameraIntrinsic[CameraID].GetCameraDistortion(Order)[i] << ";  ";
							}
							phoc << std::endl;
							}*/
							for (int Order = 0; Order < CONTROL_POINTS_ORDER + 1; ++Order) {
								phoc("Camera # params: FOV: [# ; #], Principal Point: [# ; #], Distortion Coefficients:", CameraID, CameraExtendedIntrinsic[CameraID].GetCameraFOV(Order)[0], CameraExtendedIntrinsic[CameraID].GetCameraFOV(Order)[1], CameraExtendedIntrinsic[CameraID].GetCameraPrincipalPoint(Order)[0], CameraExtendedIntrinsic[CameraID].GetCameraPrincipalPoint(Order)[1]);
								for (int i = 0; i < DistortionCoefficientsCount; ++i) {
									phoc << CameraExtendedIntrinsic[CameraID].GetCameraDistortion(Order)[i] << ";  ";
								}
								phoc << std::endl;
							}
						}
						for (std::size_t CameraID = 0; CameraID < CamerasCount; ++CameraID) {
							cv::imwrite(phoc.ToString("Output/ErrorsCamera(#).png", CameraID), DebugImages[CameraID]);
							for (std::size_t PatternID = 0; PatternID < PerPatternDebugImage[CameraID].size(); ++PatternID) {
								boost::filesystem::create_directories(phoc.ToString("Output/Camera#PerPatternErrors", CameraID));
								Phoio::imWrite(phoc.ToString("Output/Camera#PerPatternErrors/ErrorsPattern(#).tif", CameraID, PatternID), PerPatternDebugImage[CameraID][PatternID]);
							}
						}

						std::vector<double> FinalPointBlursSorted[2];
						FinalPointBlursSorted[0] = FinalPointBlurs[0];
						std::sort(FinalPointBlursSorted[0].begin(), FinalPointBlursSorted[0].end());
						FinalPointBlursSorted[1] = FinalPointBlurs[1];
						std::sort(FinalPointBlursSorted[1].begin(), FinalPointBlursSorted[1].end());

						/*MaxBlurs[0] = FinalPointBlursSorted[0][FinalPointBlursSorted[0].size() - 1 - FinalPointBlursSorted[0].size() / 9];
						MaxBlurs[1] = FinalPointBlursSorted[1][FinalPointBlursSorted[1].size() - 1 - FinalPointBlursSorted[1].size() / 9];*/

						MaxBlurs[0] = FinalPointBlursSorted[0].back();
						MaxBlurs[1] = FinalPointBlursSorted[1].back();

						double MinBlurs[2];
						MinBlurs[0] = FinalPointBlursSorted[0].front();
						MinBlurs[1] = FinalPointBlursSorted[1].front();

						{
							phoc("FinalPatternPoints3D.size = #; FinalReprojectionErrors3D.size = #", FinalPatternPoints3D.size(), FinalReprojectionErrors3D.size());
							PLogDefault;



							cv::Mat PatternPointsMat(cv::Size(1, FinalPatternPoints3D.size()), CV_32FC3, cv::Scalar(0.0, 0.0, 0.0));
							PLogMsgL;
							cv::Mat ErrorsMat(cv::Size(1, FinalReprojectionErrors3D.size()), CV_32FC3, cv::Scalar(0.0, 0.0, 0.0));
							cv::Mat ReconstructedPointsMat(cv::Size(1, FinalReprojectionErrors3D.size()), CV_32FC3, cv::Scalar(0.0, 0.0, 0.0));
							cv::Mat ReconstructedPointsMatNormal(cv::Size(1, FinalReprojectionErrors3D.size()), CV_32FC3, cv::Scalar(0.0, 0.0, 0.0));
							cv::Mat ReconstructedPointsMatWarped(cv::Size(1, FinalReprojectionErrors3D.size()), CV_32FC3, cv::Scalar(0.0, 0.0, 0.0));
							std::vector<cv::Point3d> ReconstructedPoints;
							std::vector<cv::Point3d> ReconstructedPointsCorrectedByWarp;

							std::vector<cv::Point3d> ReconstructedErrors, ReconstructedWarpedErrors;




							cv::Mat TransformationCrossMat(cv::Size(1, FinalPatternTransformations.size() * 2002 + ObjectPointSet.size()), CV_32FC3, cv::Scalar(0.0, 0.0, 0.0));
							cv::Mat TransformationCrossTexture(TransformationCrossMat.size(), CV_8UC3, cv::Scalar(0.0, 255.0, 0.0));
							int IndexTemp = 0;
							double DistanceTop, DistanceBottom, DistanceLeft, DistanceRight, DistanceMiddle = 0.0;
							DistanceTop = cv::norm(FinalPatternTransformations.back().inv().transformPoint(cv::Point3d(0.0, 0.0, 0.0) + (double)500.0 * cv::Point3d(0.0, 100.0, 0.0)) - FinalPatternTransformations.front().inv().transformPoint(cv::Point3d(0.0, 0.0, 0.0) + (double)500.0 * cv::Point3d(0.0, 100.0, 0.0)));
							DistanceBottom = cv::norm(FinalPatternTransformations.back().inv().transformPoint(cv::Point3d(0.0, 0.0, 0.0) - (double)500.0 * cv::Point3d(0.0, 100.0, 0.0)) - FinalPatternTransformations.front().inv().transformPoint(cv::Point3d(0.0, 0.0, 0.0) - (double)500.0 * cv::Point3d(0.0, 100.0, 0.0)));
							DistanceRight = cv::norm(FinalPatternTransformations.back().inv().transformPoint(cv::Point3d(0.0, 0.0, 0.0) + (double)500.0 * cv::Point3d(100.0, 0.0, 0.0)) - FinalPatternTransformations.front().inv().transformPoint(cv::Point3d(0.0, 0.0, 0.0) + (double)500.0 * cv::Point3d(100.0, 0.0, 0.0)));
							DistanceLeft = cv::norm(FinalPatternTransformations.back().inv().transformPoint(cv::Point3d(0.0, 0.0, 0.0) - (double)500.0 * cv::Point3d(100.0, 0.0, 0.0)) - FinalPatternTransformations.front().inv().transformPoint(cv::Point3d(0.0, 0.0, 0.0) - (double)500.0 * cv::Point3d(100.0, 0.0, 0.0)));
							DistanceMiddle = cv::norm(FinalPatternTransformations.back().inv().transformPoint(cv::Point3d(0.0, 0.0, 0.0)) - FinalPatternTransformations.front().inv().transformPoint(cv::Point3d(0.0, 0.0, 0.0)));
							double DistanceStrange = cv::norm(FinalPatternTransformations.back().inv().transformPoint(cv::Point3d(-300.0, 1000.0, 0.0)) - FinalPatternTransformations.front().inv().transformPoint(cv::Point3d(-300.0, 1000.0, 0.0)));



							phoc("ObjectPointSet.size() = #", ObjectPointSet.size());
							for (auto iterator = ObjectPointSet.begin(); iterator != ObjectPointSet.end(); ++iterator) {
								std::tuple<double, double, double> CurrentPatternPoint = *iterator;
								TransformationCrossMat.at<cv::Point3f>(IndexTemp++, 0) = (cv::Point3f)(cv::Point3d)(std::get<0>(CurrentPatternPoint), std::get<1>(CurrentPatternPoint), std::get<2>(CurrentPatternPoint));
							}
							for (std::size_t TransformationID = 0; TransformationID < FinalPatternTransformations.size(); ++TransformationID) {
								pho::Transformation3D64 Transformation = FinalPatternTransformations[TransformationID].inv();
								for (int k = -500; k <= 500; k++) {
									TransformationCrossMat.at<cv::Point3f>(IndexTemp++, 0) = (cv::Point3f)Transformation.transformPoint(cv::Point3d(0.0, 0.0, 0.0) + (double)k * cv::Point3d(100.0, 0.0, 0.0));
									TransformationCrossMat.at<cv::Point3f>(IndexTemp++, 0) = (cv::Point3f)Transformation.transformPoint(cv::Point3d(0.0, 0.0, 0.0) + (double)k * cv::Point3d(0.0, 100.0, 0.0));
								}
							}




							const int Order = 3;
							double Offset = 75.0;


							PLogMsgL;

							//cv::Point3d MaxError;
							double MaxErrorValue = 0.0;

							for (std::size_t PointID = 0; PointID < FinalPatternPoints3D.size(); PointID++) {
								PatternPointsMat.at<cv::Point3f>(PointID, 0) = (cv::Point3f)FinalPatternPoints3D[PointID];
								ErrorsMat.at<cv::Point3f>(PointID, 0) = (cv::Point3f)FinalReprojectionErrors3D[PointID];
								ReconstructedPointsMat.at<cv::Point3f>(PointID, 0) = (cv::Point3f)FinalPatternPoints3D[PointID] - (cv::Point3f)FinalReprojectionErrors3D[PointID] * 33.3333;
								ReconstructedPointsMatNormal.at<cv::Point3f>(PointID, 0) = (cv::Point3f)FinalPatternPoints3D[PointID] - (cv::Point3f)FinalReprojectionErrors3D[PointID];
								ReconstructedPoints.push_back(FinalPatternPoints3D[PointID] - FinalReprojectionErrors3D[PointID]);

								ReconstructedErrors.push_back(FinalPatternPoints3D[PointID] - ReconstructedPoints[PointID]);

								double ActualErrorValue = cv::norm(ReconstructedErrors.back());
								if (ActualErrorValue > MaxErrorValue) {
									MaxErrorValue = ActualErrorValue;
								}
							}
							ReconstructedPointsCorrectedByWarp = ReconstructedPoints;


							double XMin = ReconstructedPoints[0].x;
							double YMin = ReconstructedPoints[0].y;
							double ZMin = ReconstructedPoints[0].z;
							double XMax = ReconstructedPoints[0].x;
							double YMax = ReconstructedPoints[0].y;
							double ZMax = ReconstructedPoints[0].z;

							for (std::size_t PointID = 0; PointID < ReconstructedPoints.size(); PointID++) {
								XMin = std::min(XMin, ReconstructedPoints[PointID].x);
								YMin = std::min(YMin, ReconstructedPoints[PointID].y);
								ZMin = std::min(ZMin, ReconstructedPoints[PointID].z);
								XMax = std::max(XMax, ReconstructedPoints[PointID].x);
								YMax = std::max(YMax, ReconstructedPoints[PointID].y);
								ZMax = std::max(ZMax, ReconstructedPoints[PointID].z);
							}

							XMin -= Offset;
							YMin -= Offset;
							ZMin -= Offset;
							XMax += Offset;
							YMax += Offset;
							ZMax += Offset;

							PLogMsgL;

							pho::Volumetric::BezierVolumeWarperPrism<double> VolumePrism(XMin, YMin, ZMin, XMax, YMax, ZMax);
							double InitialReprojectionError, InitialReprojectionErrorMSQR, FinalReprojectionError, FinalReprojectionErrorMSQR;
							VolumePrism.FitVolume(FinalPatternPoints3D, ReconstructedPoints, Order, InitialReprojectionError, FinalReprojectionError, InitialReprojectionErrorMSQR, FinalReprojectionErrorMSQR, 2);
							PLogMsgL;

							phoc("Initial Error: Average(#), MSQR(#)", InitialReprojectionError, InitialReprojectionErrorMSQR);
							phoc("Final Error: Average(#), MSQR(#)", FinalReprojectionError, FinalReprojectionErrorMSQR);

							VolumePrism.WarpPointsInVector(ReconstructedPointsCorrectedByWarp);

							for (std::size_t PointID = 0; PointID < ReconstructedPoints.size(); PointID++) {
								ReconstructedPointsMatWarped.at<cv::Point3f>(PointID, 0) = (cv::Point3f)FinalPatternPoints3D[PointID] - (cv::Point3f)((FinalPatternPoints3D[PointID] - ReconstructedPointsCorrectedByWarp[PointID]) * 33.3333);

								ReconstructedWarpedErrors.push_back(FinalPatternPoints3D[PointID] - ReconstructedPointsCorrectedByWarp[PointID]);

								double ActualErrorValue = cv::norm(ReconstructedWarpedErrors.back());
								if (ActualErrorValue > MaxErrorValue) {
									MaxErrorValue = ActualErrorValue;
								}
							}
							/*cv::Mat ErrorsMat(FinalReprojectionErrors3D);
							cv::Mat ReconstructedPointsMat = PatternPointsMat + ErrorsMat;

							cv::Mat PatternPointsMat(FinalPatternPoints3D);
							cv::Mat ErrorsMat(FinalReprojectionErrors3D);
							cv::Mat ReconstructedPointsMat = PatternPointsMat + ErrorsMat;*/


							PLogMsgL;
							//cv::Mat Robot(cv::Size(Sphere.size(), RobotCoordinates.size()), CV_32FC3, cv::Scalar(0.0, 0.0, 0.0));
							cv::Mat PatternPointsTexture(PatternPointsMat.size(), CV_8UC3, cv::Scalar(0.0, 255.0, 0.0));
							PLogMsgL;
							//cv::Mat ICPTransform(cv::Size(Sphere.size(), TransformedICPCoordinates.size()), CV_32FC3, cv::Scalar(0.0, 0.0, 0.0));
							cv::Mat ReconstructedPointsTexture(ReconstructedPointsMat.size(), CV_8UC3, cv::Scalar(0.0, 0.0, 255.0));
							PLogMsgL;
							cv::Mat ReconstructedPointsTextureWarped(ReconstructedPointsMatWarped.size(), CV_8UC3, cv::Scalar(255.0, 0.0, 0.0));
							PLogMsgL;

							std::vector<cv::Mat> PatternPointsErrorsTextures(6);
							for (std::size_t TextureID = 0; TextureID < 3; ++TextureID) {
								PatternPointsErrorsTextures[TextureID] = cv::Mat(ReconstructedPointsMat.size(), CV_8UC3, cv::Scalar(0.0, 0.0, 255.0));
							}
							for (std::size_t TextureID = 3; TextureID < 6; ++TextureID) {
								PatternPointsErrorsTextures[TextureID] = cv::Mat(ReconstructedPointsMatWarped.size(), CV_8UC3, cv::Scalar(255.0, 0.0, 0.0));
							}

							cv::Mat Cam0BlurTexture(PatternPointsMat.size(), CV_8UC3, cv::Scalar(0.0, 0.0, 0.0));
							cv::Mat Cam1BlurTexture(PatternPointsMat.size(), CV_8UC3, cv::Scalar(0.0, 0.0, 0.0));

							int LocalIndex = 0;
							for (int y = 0; y < PatternPointsMat.rows; ++y) {
								for (int x = 0; x < PatternPointsMat.cols; ++x) {
									double ActualBlur = FinalPointBlurs[0][LocalIndex];
									ActualBlur = (ActualBlur - MinBlurs[0]) / (MaxBlurs[0] - MinBlurs[0]);
									ActualBlur *= 3;
									Cam0BlurTexture.at<cv::Vec3b>(y, x) = cv::Vec3b((int)(ActualBlur * 255.0) % 256, (int)(ActualBlur * 255.0) % 256, (int)(ActualBlur * 255.0) % 256);

									ActualBlur = FinalPointBlurs[1][LocalIndex];
									ActualBlur = (ActualBlur - MinBlurs[1]) / (MaxBlurs[1] - MinBlurs[1]);
									ActualBlur *= 3;
									Cam1BlurTexture.at<cv::Vec3b>(y, x) = cv::Vec3b((int)(ActualBlur * 255.0) % 256, (int)(ActualBlur * 255.0) % 256, (int)(ActualBlur * 255.0) % 256);

									cv::Point3d ActError = ReconstructedErrors[LocalIndex] * (1.0 / MaxErrorValue);
									ActError.x = std::abs(ActError.x);
									ActError.y = std::abs(ActError.y);
									ActError.z = std::abs(ActError.z);

									PatternPointsErrorsTextures[0].at<cv::Vec3b>(y, x) = cv::Vec3b(ActError.x * 255.0, ActError.x * 255.0, ActError.x * 255.0);
									PatternPointsErrorsTextures[1].at<cv::Vec3b>(y, x) = cv::Vec3b(ActError.y * 255.0, ActError.y * 255.0, ActError.y * 255.0);
									PatternPointsErrorsTextures[2].at<cv::Vec3b>(y, x) = cv::Vec3b(ActError.z * 255.0, ActError.z * 255.0, ActError.z * 255.0);

									ActError = ReconstructedWarpedErrors[LocalIndex] * (1.0 / MaxErrorValue);
									ActError.x = std::abs(ActError.x);
									ActError.y = std::abs(ActError.y);
									ActError.z = std::abs(ActError.z);

									PatternPointsErrorsTextures[3].at<cv::Vec3b>(y, x) = cv::Vec3b(ActError.x * 255.0, ActError.x * 255.0, ActError.x * 255.0);
									PatternPointsErrorsTextures[4].at<cv::Vec3b>(y, x) = cv::Vec3b(ActError.y * 255.0, ActError.y * 255.0, ActError.y * 255.0);
									PatternPointsErrorsTextures[5].at<cv::Vec3b>(y, x) = cv::Vec3b(ActError.z * 255.0, ActError.z * 255.0, ActError.z * 255.0);

									++LocalIndex;

								}
							}

							/*for (std::size_t SphereIndex = 0; SphereIndex < RobotCoordinates.size(); SphereIndex++) {
							for (std::size_t PointIndex = 0; PointIndex < Sphere.size(); PointIndex++) {
							Robot.at<cv::Point3f>(SphereIndex, PointIndex) = (cv::Point3f)RobotCoordinates[SphereIndex] + Sphere[PointIndex];
							ICPTransform.at<cv::Point3f>(SphereIndex, PointIndex) = (cv::Point3f)TransformedICPCoordinates[SphereIndex] + Sphere[PointIndex];
							}
							}*/

							PLogMsgL;
							pho::PointCloud PatternPointsCloud(PatternPointsMat, PatternPointsTexture);

							pho::PointCloud PatternPointsCloudBlur0(PatternPointsMat, Cam0BlurTexture);
							pho::PointCloud PatternPointsCloudBlur1(PatternPointsMat, Cam1BlurTexture);

							pho::PointCloud ReconstructedPointsCloud(ReconstructedPointsMat, ReconstructedPointsTexture);
							pho::PointCloud ReconstructedPointsCloudNormal(ReconstructedPointsMatNormal, ReconstructedPointsTexture);
							pho::PointCloud ReconstructedPointsCloudWarped(ReconstructedPointsMatWarped, ReconstructedPointsTextureWarped);

							pho::PointCloud ReconstructedPointsCloudX(ReconstructedPointsMat, PatternPointsErrorsTextures[0]);
							pho::PointCloud ReconstructedPointsCloudY(ReconstructedPointsMat, PatternPointsErrorsTextures[1]);
							pho::PointCloud ReconstructedPointsCloudZ(ReconstructedPointsMat, PatternPointsErrorsTextures[2]);
							pho::PointCloud ReconstructedPointsCloudWarpedX(ReconstructedPointsMatWarped, PatternPointsErrorsTextures[3]);
							pho::PointCloud ReconstructedPointsCloudWarpedY(ReconstructedPointsMatWarped, PatternPointsErrorsTextures[4]);
							pho::PointCloud ReconstructedPointsCloudWarpedZ(ReconstructedPointsMatWarped, PatternPointsErrorsTextures[5]);

							pho::PointCloud TransformationCrossCloud(TransformationCrossMat, TransformationCrossTexture);
							std::ofstream TransformationCrossCloudBorderPointsDistances("Output/TransformationCrossBorderPointsDistances.txt");
							if (TransformationCrossCloudBorderPointsDistances.is_open()) {
								TransformationCrossCloudBorderPointsDistances << phoc.ToString("DistanceMiddle: #", DistanceMiddle) << std::endl;
								TransformationCrossCloudBorderPointsDistances << phoc.ToString("DistanceRight: #", DistanceRight) << std::endl;
								TransformationCrossCloudBorderPointsDistances << phoc.ToString("DistanceLeft: #", DistanceLeft) << std::endl;
								TransformationCrossCloudBorderPointsDistances << phoc.ToString("DistanceTop: #", DistanceTop) << std::endl;
								TransformationCrossCloudBorderPointsDistances << phoc.ToString("DistanceBottom: #", DistanceBottom) << std::endl;
								TransformationCrossCloudBorderPointsDistances << phoc.ToString("DistanceStrange: #", DistanceStrange) << std::endl;
								TransformationCrossCloudBorderPointsDistances.close();
							}

							PLogMsgL;

							pho::Plyio::plyWrite("Output/PatternPoints.ply", PatternPointsCloud);
							pho::Plyio::plyWrite("Output/PatternPointsCloudBlur0.ply", PatternPointsCloudBlur0);
							pho::Plyio::plyWrite("Output/PatternPointsCloudBlur1.ply", PatternPointsCloudBlur1);
							PLogMsgL;
							pho::Plyio::plyWrite("Output/ReconstructedPoints.ply", ReconstructedPointsCloud);
							pho::Plyio::plyWrite("Output/ReconstructedPointsNormal.ply", ReconstructedPointsCloudNormal);
							pho::Plyio::plyWrite("Output/ReconstructedPointsX.ply", ReconstructedPointsCloudX);
							pho::Plyio::plyWrite("Output/ReconstructedPointsY.ply", ReconstructedPointsCloudY);
							pho::Plyio::plyWrite("Output/ReconstructedPointsZ.ply", ReconstructedPointsCloudZ);
							PLogMsgL;
							pho::Plyio::plyWrite("Output/ReconstructedPointsWarped.ply", ReconstructedPointsCloudWarped);
							pho::Plyio::plyWrite("Output/ReconstructedPointsWarpedX.ply", ReconstructedPointsCloudWarpedX);
							pho::Plyio::plyWrite("Output/ReconstructedPointsWarpedY.ply", ReconstructedPointsCloudWarpedY);
							pho::Plyio::plyWrite("Output/ReconstructedPointsWarpedZ.ply", ReconstructedPointsCloudWarpedZ);

							pho::Plyio::plyWrite("Output/TransformationCross.ply", TransformationCrossCloud);
							PLogMsgL;
						}

					}
				}


#if 0
				ceres::Problem problem;

				for (std::size_t CameraID = 0; CameraID < CameraRotationQuaternions.size(); ++CameraID) {
					problem.AddParameterBlock(CameraRotationQuaternions[CameraID].val, 4, new ceres::QuaternionParameterization());
				}
				for (std::size_t PatternID = 0; PatternID < PatternRotationQuaternions.size(); ++PatternID) {
					problem.AddParameterBlock(PatternRotationQuaternions[PatternID].val, 4, new ceres::QuaternionParameterization());
				}
				for (std::size_t PointID = 0; PointID < AllCorrespondences.size(); ++PointID) {
					cv::Vec2i ID = AllCorrespondences[PointID].first;
					int CameraID = ID.val[0];
					int PatternID = ID.val[1];
					ceres::CostFunction* cost_function = MeasuredCalibrationPointWithDistortionWarpFunctor<DistortionCoefficientsCount, CONTROL_POINTS_ORDER>::Create(AllCorrespondences[PointID].second.second, AllCorrespondences[PointID].second.first, FinalReprojectionErrorsAndObjectPoints[PointID].second.second, FinalReprojectionErrorsAndObjectPoints[PointID].second.first, MicroDistortionWarpers[CameraID]);
					problem.AddResidualBlock(cost_function,
						NULL /* squared loss */,
						CameraIntrinsic[CameraID].GetData(),
						/*CameraFOVs[CameraID].val,
						CameraPrincipals[CameraID].val,
						CameraDistortionsControlPoints[CameraID][0].val,*/
						PatternTranslationVectors[PatternID].val,
						PatternRotationQuaternions[PatternID].val,
						CameraTranslationVectors[CameraID].val,
						CameraRotationQuaternions[CameraID].val);
				}

				problem.SetParameterBlockConstant(CameraTranslationVectors[0].val);
				problem.SetParameterBlockConstant(CameraRotationQuaternions[0].val);

				/*for (std::size_t CameraID = 0; CameraID < CameraRotationQuaternions.size(); ++CameraID) {
				problem.SetParameterBlockConstant(CameraTranslationVectors[CameraID].val);
				problem.SetParameterBlockConstant(CameraRotationQuaternions[CameraID].val);
				problem.SetParameterBlockConstant(CameraFOVs[CameraID].val);
				problem.SetParameterBlockConstant(CameraPrincipals[CameraID].val);
				}*/

				ceres::Solver::Options options;
				options.num_threads = 8;
				//options.sparse_linear_algebra_library_type = ceres::CX_SPARSE;
				//options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
				options.linear_solver_type = ceres::DENSE_SCHUR;
				options.max_num_iterations = 5000;
				options.minimizer_progress_to_stdout = true;
				ceres::Solver::Summary summary;
				ceres::Solve(options, &problem, &summary);
				std::cout << summary.FullReport() << std::endl;

				{
					//Own Evaluation for All Points
					std::vector<cv::Mat> DebugImages(CamerasCount);
					for (std::size_t CameraID = 0; CameraID < CamerasCount; ++CameraID) {
						DebugImages[CameraID] = cv::Mat(CameraResolutions[CameraID], CV_8UC3, cv::Scalar(0.0, 0.0, 0.0));
					}
					std::vector<MeasuredCalibrationPointWithDistortionWarpFunctor<DistortionCoefficientsCount, CONTROL_POINTS_ORDER>> Functors;
					for (std::size_t PointID = 0; PointID < AllCorrespondencesOriginal.size(); ++PointID) {
						cv::Vec2i ID = AllCorrespondencesOriginal[PointID].first;
						int CameraID = ID.val[0];
						int PatternID = ID.val[1];

						Functors.push_back(MeasuredCalibrationPointWithDistortionWarpFunctor<DistortionCoefficientsCount, CONTROL_POINTS_ORDER>(AllCorrespondencesOriginal[PointID].second.second, AllCorrespondencesOriginal[PointID].second.first, FinalReprojectionErrorsAndObjectPointsOriginal[PointID].second.second, FinalReprojectionErrorsAndObjectPointsOriginal[PointID].second.first, MicroDistortionWarpers[CameraID]));

						double residual[2];
						Functors[PointID](
							CameraIntrinsic[CameraID].GetData(),
							/*CameraFOVs[CameraID].val,
							CameraPrincipals[CameraID].val,
							CameraDistortionsControlPoints[CameraID][0].val,*/
							PatternTranslationVectors[PatternID].val,
							PatternRotationQuaternions[PatternID].val,
							CameraTranslationVectors[CameraID].val,
							CameraRotationQuaternions[CameraID].val,
							residual);

						double ActualError = cv::norm(FinalReprojectionErrorsAndObjectPointsOriginal[PointID].second.first);

						double MaxError = 0.693018;
						ActualError *= (1.0)* 0.693018;

						double ActualErrorNormalized = std::min(std::max(ActualError, 0.0), 1.0);
						//cv::circle(DebugImages[CameraID], AllCorrespondencesOriginal[PointID].second.first, 4, cv::Scalar(0.0, 255.0 * (1.0 - ActualErrorNormalized), 255.0 * ActualErrorNormalized), -1);

					}




					{
						std::vector<std::pair<double, int>> ReprojectionErrorsIndexesFinal;
						for (std::size_t PointID = 0; PointID < FinalReprojectionErrorsAndObjectPointsOriginal.size(); ++PointID) {
							ReprojectionErrorsIndexesFinal.push_back(std::pair<double, int>(cv::norm(FinalReprojectionErrorsAndObjectPointsOriginal[PointID].second.first), (int)PointID));
						}
						std::sort(ReprojectionErrorsIndexesFinal.begin(), ReprojectionErrorsIndexesFinal.end());

						std::ofstream ReprojectionErrorsIndexesStream("Output/ReprojectionErrorsIndexesFinal.txt");

						double MaxError = cv::norm(FinalReprojectionErrorsAndObjectPointsOriginal[ReprojectionErrorsIndexesFinal[0].second].second.first);
						for (std::size_t i = 1; i < ReprojectionErrorsIndexesFinal.size(); ++i) {
							double ActualError = cv::norm(FinalReprojectionErrorsAndObjectPointsOriginal[ReprojectionErrorsIndexesFinal[i].second].second.first);
							MaxError = std::max(ActualError, MaxError);
						}

						for (std::size_t i = 0; i < ReprojectionErrorsIndexesFinal.size(); ++i) {

							double ActualError = cv::norm(FinalReprojectionErrorsAndObjectPointsOriginal[ReprojectionErrorsIndexesFinal[i].second].second.first);
							ActualError /= MaxError;
							double ActualErrorNormalized = std::min(std::max(ActualError, 0.0), 1.0);
							cv::circle(DebugImages[AllCorrespondencesOriginal[ReprojectionErrorsIndexesFinal[i].second].first.val[0]], AllCorrespondencesOriginal[ReprojectionErrorsIndexesFinal[i].second].second.first, 10, cv::Scalar(0.0, 255.0 * (1.0 - ActualErrorNormalized), 255.0 * ActualErrorNormalized), -1);

							ReprojectionErrorsIndexesStream << ReprojectionErrorsIndexesFinal[i].first << std::endl;
						}

						ReprojectionErrorsIndexesStream.close();
					}

					for (std::size_t CameraID = 0; CameraID < CamerasCount; ++CameraID) {
						cv::imwrite(phoc.ToString("Output/ErrorsCamera(#).png", CameraID), DebugImages[CameraID]);
					}
				}


				for (std::size_t PointID = 0; PointID < FinalReprojectionErrorsAndObjectPoints.size(); ++PointID) {
					cv::Vec2i ID = FinalReprojectionErrorsAndObjectPoints[PointID].first;
					int CameraID = ID.val[0];
					int PatternID = ID.val[1];
					double& MinDepth = MinDepths[CameraID];
					double& MaxDepth = MaxDepths[CameraID];
					cv::Point2d& MaxReprojectionError = MaxReprojectionErrors[CameraID];
					cv::Point3d& ActualPoint = FinalReprojectionErrorsAndObjectPoints[PointID].second.second;
					cv::Point2d& ActualReprojectionError = FinalReprojectionErrorsAndObjectPoints[PointID].second.first;


					++Points[CameraID];
					AverageReprojectionError[CameraID] += cv::norm(ActualReprojectionError);

					if (MinDepth == 0.0) MinDepth = ActualPoint.z;
					if (MaxDepth == 0.0) MaxDepth = ActualPoint.z;
					if (MaxReprojectionError == cv::Point2d(0.0, 0.0)) MaxReprojectionError = ActualReprojectionError;

					if (ActualPoint.z < MinDepth) MinDepth = ActualPoint.z;

					if (ActualPoint.z > MaxDepth) MaxDepth = ActualPoint.z;

					if (ActualReprojectionError.x > MaxReprojectionError.x) MaxReprojectionError.x = ActualReprojectionError.x;
					if (ActualReprojectionError.y > MaxReprojectionError.y) MaxReprojectionError.y = ActualReprojectionError.y;

				}

				for (std::size_t CameraID = 0; CameraID < CamerasCount; ++CameraID) {
					if (Points[CameraID]) AverageReprojectionError[CameraID] /= (double)Points[CameraID];
					phoc("Final Camera #: MinDepth( # ); MaxDepth( # ); MaxReprojectionError( # ); AverageReprojectionError( # )", CameraID, MinDepths[CameraID], MaxDepths[CameraID], MaxReprojectionErrors[CameraID], AverageReprojectionError[CameraID]);
				}

				boost::this_thread::sleep_for(boost::chrono::milliseconds(5000));

				if (!summary.IsSolutionUsable()) {
					return false;// -1.0;
				}
#endif
			}

#if 0
			if (0) {
				std::vector<std::unique_ptr<pho::Volumetric::BezierSpecificVolumeWarperPrism<3, 3, 3>>> VolumeWarpers(CamerasCount);
				std::vector<MicroBezierWarper<4>> MicroWarpers;
				std::vector<cv::Point3d> MinPoints(CamerasCount, cv::Point3d(0.0, 0.0, 0.0));
				std::vector<cv::Point3d> MaxPoints(CamerasCount, cv::Point3d(0.0, 0.0, 0.0));

				std::vector<cv::Point2d> MaxReprojectionErrors(CamerasCount, cv::Point2d(0.0, 0.0));
				std::vector<int> Points(CamerasCount, 0);
				std::vector<double> AverageReprojectionError(CamerasCount, 0.0);

				for (std::size_t PointID = 0; PointID < FinalReprojectionErrorsAndObjectPoints.size(); ++PointID) {
					cv::Vec2i ID = FinalReprojectionErrorsAndObjectPoints[PointID].first;
					int CameraID = ID.val[0];
					int PatternID = ID.val[1];
					cv::Point3d& MinPoint = MinPoints[CameraID];
					cv::Point3d& MaxPoint = MaxPoints[CameraID];
					cv::Point2d& MaxReprojectionError = MaxReprojectionErrors[CameraID];
					cv::Point3d& ActualPoint = FinalReprojectionErrorsAndObjectPoints[PointID].second.second;
					cv::Point2d& ActualReprojectionError = FinalReprojectionErrorsAndObjectPoints[PointID].second.first;


					++Points[CameraID];
					AverageReprojectionError[CameraID] += cv::norm(ActualReprojectionError);

					if (MinPoint == cv::Point3d(0.0, 0.0, 0.0)) MinPoint = ActualPoint;
					if (MaxPoint == cv::Point3d(0.0, 0.0, 0.0)) MaxPoint = ActualPoint;
					if (MaxReprojectionError == cv::Point2d(0.0, 0.0)) MaxReprojectionError = ActualReprojectionError;

					if (ActualPoint.x < MinPoint.x) MinPoint.x = ActualPoint.x;
					if (ActualPoint.y < MinPoint.y) MinPoint.y = ActualPoint.y;
					if (ActualPoint.z < MinPoint.z) MinPoint.z = ActualPoint.z;

					if (ActualPoint.x > MaxPoint.x) MaxPoint.x = ActualPoint.x;
					if (ActualPoint.y > MaxPoint.y) MaxPoint.y = ActualPoint.y;
					if (ActualPoint.z > MaxPoint.z) MaxPoint.z = ActualPoint.z;

					if (ActualReprojectionError.x > MaxReprojectionError.x) MaxReprojectionError.x = ActualReprojectionError.x;
					if (ActualReprojectionError.y > MaxReprojectionError.y) MaxReprojectionError.y = ActualReprojectionError.y;

				}

				double offset = 100.0;

				for (std::size_t CameraID = 0; CameraID < CamerasCount; ++CameraID) {
					if (Points[CameraID]) AverageReprojectionError[CameraID] /= (double)Points[CameraID];

					phoc("Camera #: MinPoint( # ); MaxPoint( # ); MaxReprojectionError( # ); AverageReprojectionError( # )", CameraID, MinPoints[CameraID], MaxPoints[CameraID], MaxReprojectionErrors[CameraID], AverageReprojectionError[CameraID]);

					AverageReprojectionError[CameraID] = 0.0;
					Points[CameraID] = 0;
					MaxReprojectionErrors[CameraID] = cv::Point2d(0.0, 0.0);

					pho::Volumetric::BezierSpecificVolumeWarperPrism<3, 3, 3>* CameraSpecificWarp = new pho::Volumetric::BezierSpecificVolumeWarperPrism<3, 3, 3>(MinPoints[CameraID].x - offset, MinPoints[CameraID].y - offset, MinPoints[CameraID].z - offset, MaxPoints[CameraID].x + offset, MaxPoints[CameraID].y + offset, MaxPoints[CameraID].z + offset);

					VolumeWarpers[CameraID] = std::unique_ptr<pho::Volumetric::BezierSpecificVolumeWarperPrism<3, 3, 3>>(CameraSpecificWarp);
					MicroWarpers.push_back(MicroBezierWarper<4>(MinPoints[CameraID].x - offset, MinPoints[CameraID].y - offset, MinPoints[CameraID].z - offset, MaxPoints[CameraID].x + offset, MaxPoints[CameraID].y + offset, MaxPoints[CameraID].z + offset));

					MaxPoints[CameraID] = cv::Point3d(0.0, 0.0, 0.0);
					MinPoints[CameraID] = cv::Point3d(0.0, 0.0, 0.0);
				}


				ceres::Problem problem;

				for (std::size_t CameraID = 0; CameraID < CameraRotationQuaternions.size(); ++CameraID) {
					problem.AddParameterBlock(CameraRotationQuaternions[CameraID].val, 4, new ceres::QuaternionParameterization());
				}

				for (std::size_t PatternID = 0; PatternID < PatternRotationQuaternions.size(); ++PatternID) {
					problem.AddParameterBlock(PatternRotationQuaternions[PatternID].val, 4, new ceres::QuaternionParameterization());
				}
				for (std::size_t PointID = 0; PointID < AllCorrespondences.size(); ++PointID) {
					cv::Vec2i ID = AllCorrespondences[PointID].first;
					int CameraID = ID.val[0];
					int PatternID = ID.val[1];
					ceres::CostFunction* cost_function = MeasuredCalibrationPointWithWarpFunctor<DistortionCoefficientsCount, 3>::Create(AllCorrespondences[PointID].second.second, AllCorrespondences[PointID].second.first, FinalReprojectionErrorsAndObjectPoints[PointID].second.second, FinalReprojectionErrorsAndObjectPoints[PointID].second.first, MicroWarpers[CameraID]);
					problem.AddResidualBlock(cost_function,
						NULL /* squared loss */,
						CameraIntrinsic[CameraID].GetData(),
						/*CameraFOVs[CameraID].val,
						CameraPrincipals[CameraID].val,
						CameraDistortions[CameraID].val,*/
						PatternTranslationVectors[PatternID].val,
						PatternRotationQuaternions[PatternID].val,
						CameraTranslationVectors[CameraID].val,
						CameraRotationQuaternions[CameraID].val,
						(double*)&VolumeWarpers[CameraID]->GetPoints()[0]);
				}

				//problem.SetParameterBlockConstant(CameraTranslationVectors[0].val);
				//problem.SetParameterBlockConstant(CameraRotationQuaternions[0].val);
				for (std::size_t CameraID = 0; CameraID < CameraRotationQuaternions.size(); ++CameraID) {
					problem.SetParameterBlockConstant(CameraTranslationVectors[CameraID].val);
					problem.SetParameterBlockConstant(CameraRotationQuaternions[CameraID].val);
					problem.SetParameterBlockConstant(CameraIntrinsic[CameraID].GetData());

					/*problem.SetParameterBlockConstant(CameraFOVs[CameraID].val);
					problem.SetParameterBlockConstant(CameraPrincipals[CameraID].val);
					problem.SetParameterBlockConstant(CameraDistortions[CameraID].val);*/
				}

				ceres::Solver::Options options;
				options.num_threads = 8;
				//options.sparse_linear_algebra_library_type = ceres::CX_SPARSE;
				//options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
				options.linear_solver_type = ceres::DENSE_SCHUR;
				options.minimizer_progress_to_stdout = true;
				ceres::Solver::Summary summary;
				ceres::Solve(options, &problem, &summary);
				std::cout << summary.BriefReport() << std::endl;

				for (std::size_t PointID = 0; PointID < FinalReprojectionErrorsAndObjectPoints.size(); ++PointID) {
					cv::Vec2i ID = FinalReprojectionErrorsAndObjectPoints[PointID].first;
					int CameraID = ID.val[0];
					int PatternID = ID.val[1];
					cv::Point3d& MinPoint = MinPoints[CameraID];
					cv::Point3d& MaxPoint = MaxPoints[CameraID];
					cv::Point2d& MaxReprojectionError = MaxReprojectionErrors[CameraID];
					cv::Point3d& ActualPoint = FinalReprojectionErrorsAndObjectPoints[PointID].second.second;
					cv::Point2d& ActualReprojectionError = FinalReprojectionErrorsAndObjectPoints[PointID].second.first;


					++Points[CameraID];
					AverageReprojectionError[CameraID] += cv::norm(ActualReprojectionError);

					if (MinPoint == cv::Point3d(0.0, 0.0, 0.0)) MinPoint = ActualPoint;
					if (MaxPoint == cv::Point3d(0.0, 0.0, 0.0)) MaxPoint = ActualPoint;
					if (MaxReprojectionError == cv::Point2d(0.0, 0.0)) MaxReprojectionError = ActualReprojectionError;

					if (ActualPoint.x < MinPoint.x) MinPoint.x = ActualPoint.x;
					if (ActualPoint.y < MinPoint.y) MinPoint.y = ActualPoint.y;
					if (ActualPoint.z < MinPoint.z) MinPoint.z = ActualPoint.z;

					if (ActualPoint.x > MaxPoint.x) MaxPoint.x = ActualPoint.x;
					if (ActualPoint.y > MaxPoint.y) MaxPoint.y = ActualPoint.y;
					if (ActualPoint.z > MaxPoint.z) MaxPoint.z = ActualPoint.z;

					if (ActualReprojectionError.x > MaxReprojectionError.x) MaxReprojectionError.x = ActualReprojectionError.x;
					if (ActualReprojectionError.y > MaxReprojectionError.y) MaxReprojectionError.y = ActualReprojectionError.y;

				}

				for (std::size_t CameraID = 0; CameraID < CamerasCount; ++CameraID) {
					if (Points[CameraID]) AverageReprojectionError[CameraID] /= (double)Points[CameraID];
					phoc("Final Camera #: MinPoint( # ); MaxPoint( # ); MaxReprojectionError( # ); AverageReprojectionError( # )", CameraID, MinPoints[CameraID], MaxPoints[CameraID], MaxReprojectionErrors[CameraID], AverageReprojectionError[CameraID]);
				}

				for (std::size_t CameraID = 0; CameraID < CamerasCount; ++CameraID) {
					phoc << std::endl;
					const std::vector<cv::Point3d>& ControlPoints = VolumeWarpers[CameraID]->GetPoints();
					for (std::size_t ControlPointID = 0; ControlPointID < ControlPoints.size(); ++ControlPointID) {
						phoc << ControlPoints[ControlPointID] << std::endl;
					}
				}

				boost::this_thread::sleep_for(boost::chrono::milliseconds(5000));
				if (!summary.IsSolutionUsable()) {
					return false;// -1.0;
				}
			}
#endif


			for (std::size_t CameraID = 0; CameraID < CamerasCount; ++CameraID) {
				cv::Mat ActualCameraMatrix = OutputCalibrations[CameraID].GetCameraMatrix();
				cv::Mat ActualDistortionCoefficientTemp = OutputCalibrations[CameraID].GetDistortionCoefficients();
				cv::Mat ActualDistortionCoefficient(cv::Size(1, DistortionCoefficientsCount), CV_64FC1, cv::Scalar(0.0));
				if (ActualDistortionCoefficientTemp.rows > DistortionCoefficientsCount) {
					ActualDistortionCoefficientTemp(cv::Rect(0, 0, 1, DistortionCoefficientsCount)).copyTo(ActualDistortionCoefficient);
				}
				else {
					if (ActualDistortionCoefficientTemp.rows < DistortionCoefficientsCount) {
						pho_runtime_error("Incompatible number of Distortion Parameters");
					}
					ActualDistortionCoefficient = ActualDistortionCoefficientTemp;
				}
				OutputCalibrations[CameraID].SetDistortionCoefficients(ActualDistortionCoefficient);

				ActualCameraMatrix.at<double>(0, 0) = CameraExtendedIntrinsic[CameraID].GetCameraFOV()[0];
				ActualCameraMatrix.at<double>(1, 1) = CameraExtendedIntrinsic[CameraID].GetCameraFOV()[1];
				ActualCameraMatrix.at<double>(0, 2) = CameraExtendedIntrinsic[CameraID].GetCameraPrincipalPoint()[0];
				ActualCameraMatrix.at<double>(1, 2) = CameraExtendedIntrinsic[CameraID].GetCameraPrincipalPoint()[1];

				for (int i = 0; i < DistortionCoefficientsCount; ++i) {
					ActualDistortionCoefficient.at<double>(i, 0) = CameraExtendedIntrinsic[CameraID].GetCameraDistortion()[i];
				}

				OutputCalibrations[CameraID].SetCameraMatrix(ActualCameraMatrix);
				OutputCalibrations[CameraID].SetDistortionCoefficients(ActualDistortionCoefficient);
			}

			for (std::size_t CameraID = 0; CameraID < CamerasCount; ++CameraID) {
				Vec3d& ActualTranslation = CameraTranslationVectors[CameraID];
				Vec4d& ActualRotation = CameraRotationQuaternions[CameraID];

				pho::Transformation3D64 CameraTransformation = pho::Transformation3D64(
					pho::Quaternion64(ActualRotation.val[0], ActualRotation.val[1], ActualRotation.val[2], ActualRotation.val[3]),
					pho::Translation3D64(ActualTranslation.val[0], ActualTranslation.val[1], ActualTranslation.val[2])
					);
				cv::Mat TransformationMatrix = CameraTransformation;
				OutputCalibrations[CameraID].SetWorldToCameraCoordinates(TransformationMatrix(cv::Rect(0, 0, 4, 3)));
			}

			/*for (std::size_t PatternID = 0; PatternID < PatternTransformations[0].size(); ++PatternID) {
			pho::Transformation3D64 PatternTransformation = PatternTransformations[0][PatternID].second;
			pho::Quaternion64 PatternTransformationsRotationQuaternion = PatternTransformation.getRotation();
			Vec3d ActualTranslation;
			Vec4d ActualRotation;
			ActualTranslation.val[0] = PatternTransformation.getTranslation().x();
			ActualTranslation.val[1] = PatternTransformation.getTranslation().y();
			ActualTranslation.val[2] = PatternTransformation.getTranslation().z();
			ActualRotation.val[0] = PatternTransformationsRotationQuaternion.w();
			ActualRotation.val[1] = PatternTransformationsRotationQuaternion.x();
			ActualRotation.val[2] = PatternTransformationsRotationQuaternion.y();
			ActualRotation.val[3] = PatternTransformationsRotationQuaternion.z();

			PatternTranslationVectors.push_back(ActualTranslation);
			PatternRotationQuaternions.push_back(ActualRotation);
			}*/



			return true; // summary.final_cost;

		}

#define DistortionCoefficientsCount 5
#define BezierOrder 0
		void CeresMultipleCamerasCalibrationTest(){
			/*int DistortionCoefficientsCount = 8;
			int BezierOrder = 3;

			std::cout << "DistortionCoefficientsCount: ";
			std::cin >> DistortionCoefficientsCount;

			std::cout << "BezierOrder: ";
			std::cin >> BezierOrder;*/
			cv::Point3f CentralPatternPoint = cv::Point3f(1080.0f, -280.0f, 0.0f);

			vectorBox<vectorBox<cv::Point3f>> ObjectPoints;
			vectorBox<vectorBox<cv::Point2f>> ImagePoints[2];
			vectorBox<vectorBox<float>> ImagePointBlurs[2];
			std::vector<cv::Size> CameraResolutions(2);

			std::vector < std::vector<std::pair<int, std::vector<std::pair<cv::Point3d, cv::Point3d>>>>> CorrespondencePairsPerCameraPerPatternInput(2);

			cv::FileStorage FS("Inputs/TemporaryCalibrationPatterns.xml", cv::FileStorage::READ);

			FS["ObjectPoints"] >> ObjectPoints;
			FS["ImagePointsCam0"] >> ImagePoints[0];
			FS["ImagePointsCam1"] >> ImagePoints[1];
			FS["ResolutionCam0"] >> CameraResolutions[0];
			FS["ResolutionCam1"] >> CameraResolutions[1];

			bool UseBlur = false;
			if (!FS["ImagePointsCam0"].empty() && !FS["ImagePointsCam1"].empty()) {
				FS["ImagePointBlursArrayCam0"] >> ImagePointBlurs[0];
				FS["ImagePointBlursArrayCam1"] >> ImagePointBlurs[1];
				UseBlur = true;
			}

			FS.release();
			for (std::size_t i = 0; i < ObjectPoints.size(); ++i) {
				if (i < 3) continue;
				//if (i >  ObjectPoints.size() - 5) continue;*/
				std::pair<int, std::vector<std::pair<cv::Point3d, cv::Point3d>>> ActualPattern[2];
				ActualPattern[0].first = (int)i - 3;
				ActualPattern[1].first = (int)i - 3;
				for (std::size_t k = 0; k < ObjectPoints[i].size(); ++k) {
					ActualPattern[0].second.push_back(std::pair<cv::Point3d, cv::Point3d>(cv::Point3d(ImagePoints[0][i][k].x, ImagePoints[0][i][k].y, (UseBlur) ? ImagePointBlurs[0][i][k] : -1.0), (cv::Point3d)(ObjectPoints[i][k] - CentralPatternPoint)));
					ActualPattern[1].second.push_back(std::pair<cv::Point3d, cv::Point3d>(cv::Point3d(ImagePoints[1][i][k].x, ImagePoints[1][i][k].y, (UseBlur) ? ImagePointBlurs[1][i][k] : -1.0), (cv::Point3d)(ObjectPoints[i][k] - CentralPatternPoint)));
				}
				CorrespondencePairsPerCameraPerPatternInput[0].push_back(ActualPattern[0]);
				CorrespondencePairsPerCameraPerPatternInput[1].push_back(ActualPattern[1]);
			}
			std::vector<photoneoTools::CameraCalibration64> OutputCalibrations(2);

			CeresMultipleCamerasCalibration<DistortionCoefficientsCount, BezierOrder>(CorrespondencePairsPerCameraPerPatternInput, CameraResolutions, OutputCalibrations);

			/*if (DistortionCoefficientsCount == 5 && BezierOrder == 0) {
			CeresMultipleCamerasCalibration<5, 0>(CorrespondencePairsPerCameraPerPatternInput, CameraResolutions, OutputCalibrations);
			} else if (DistortionCoefficientsCount == 5 && BezierOrder == 1) {
			CeresMultipleCamerasCalibration<5, 1>(CorrespondencePairsPerCameraPerPatternInput, CameraResolutions, OutputCalibrations);
			} else if (DistortionCoefficientsCount == 5 && BezierOrder == 2) {
			CeresMultipleCamerasCalibration<5, 2>(CorrespondencePairsPerCameraPerPatternInput, CameraResolutions, OutputCalibrations);
			} else if (DistortionCoefficientsCount == 5 && BezierOrder == 3) {
			CeresMultipleCamerasCalibration<5, 3>(CorrespondencePairsPerCameraPerPatternInput, CameraResolutions, OutputCalibrations);
			} else if (DistortionCoefficientsCount == 5 && BezierOrder == 4) {
			CeresMultipleCamerasCalibration<5, 4>(CorrespondencePairsPerCameraPerPatternInput, CameraResolutions, OutputCalibrations);
			} else if (DistortionCoefficientsCount == 8 && BezierOrder == 0) {
			CeresMultipleCamerasCalibration<8, 0>(CorrespondencePairsPerCameraPerPatternInput, CameraResolutions, OutputCalibrations);
			} else if (DistortionCoefficientsCount == 8 && BezierOrder == 1) {
			CeresMultipleCamerasCalibration<8, 1>(CorrespondencePairsPerCameraPerPatternInput, CameraResolutions, OutputCalibrations);
			} else if (DistortionCoefficientsCount == 8 && BezierOrder == 2) {
			CeresMultipleCamerasCalibration<8, 2>(CorrespondencePairsPerCameraPerPatternInput, CameraResolutions, OutputCalibrations);
			} else if (DistortionCoefficientsCount == 8 && BezierOrder == 3) {
			CeresMultipleCamerasCalibration<8, 3>(CorrespondencePairsPerCameraPerPatternInput, CameraResolutions, OutputCalibrations);
			} else if (DistortionCoefficientsCount == 8 && BezierOrder == 4) {
			CeresMultipleCamerasCalibration<8, 4>(CorrespondencePairsPerCameraPerPatternInput, CameraResolutions, OutputCalibrations);
			} else if (DistortionCoefficientsCount == 12 && BezierOrder == 0) {
			CeresMultipleCamerasCalibration<12, 0>(CorrespondencePairsPerCameraPerPatternInput, CameraResolutions, OutputCalibrations);
			} else if (DistortionCoefficientsCount == 12 && BezierOrder == 1) {
			CeresMultipleCamerasCalibration<12, 1>(CorrespondencePairsPerCameraPerPatternInput, CameraResolutions, OutputCalibrations);
			} else if (DistortionCoefficientsCount == 12 && BezierOrder == 2) {
			CeresMultipleCamerasCalibration<12, 2>(CorrespondencePairsPerCameraPerPatternInput, CameraResolutions, OutputCalibrations);
			} else if (DistortionCoefficientsCount == 12 && BezierOrder == 3) {
			CeresMultipleCamerasCalibration<12, 3>(CorrespondencePairsPerCameraPerPatternInput, CameraResolutions, OutputCalibrations);
			} else if (DistortionCoefficientsCount == 12 && BezierOrder == 4) {
			CeresMultipleCamerasCalibration<12, 4>(CorrespondencePairsPerCameraPerPatternInput, CameraResolutions, OutputCalibrations);
			}*/

			cv::FileStorage FS2("Inputs/CalibratioOutput.xml", cv::FileStorage::WRITE);
			FS2 << "Calibrations" << OutputCalibrations;
			FS2.release();
		}


#undef DistortionCoefficientsCount

#if 0
		double FindRobotScannerTransformation(const std::vector<pho::Transformation3D64>& TCPToRobotSpaceTransformations, const std::vector<cv::Point3d>& RecognizedSphereCentersInScannerCoordinates, pho::Transformation3D64& FinalScannerToRobotTransformation, cv::Point3d& SphereCenterInTCPCoordinates) {

			if (TCPToRobotSpaceTransformations.size() != RecognizedSphereCentersInScannerCoordinates.size()) return -1.0;
			std::vector<cv::Point3d> InitialPositionGuesses(TCPToRobotSpaceTransformations.size());
			for (std::size_t i = 0; i < InitialPositionGuesses.size(); i++) {
				InitialPositionGuesses[i] = TCPToRobotSpaceTransformations[i].transformPoint(SphereCenterInTCPCoordinates);
			}

			pho::Transformation3D64 InitialTransformation = pho::getTransformation3D_mine(RecognizedSphereCentersInScannerCoordinates, InitialPositionGuesses);
			pho::Quaternion64 InitialTransformationRotationQuaternion = InitialTransformation.getRotation();

			double ReprojectionErrorInitial = 0.0;
			for (std::size_t i = 0; i < RecognizedSphereCentersInScannerCoordinates.size(); i++) {
				ReprojectionErrorInitial += std::pow(cv::norm(InitialTransformation.transformPoint(RecognizedSphereCentersInScannerCoordinates[i]) - InitialPositionGuesses[i]), 2.0);;
			}
			ReprojectionErrorInitial *= (1.0) / (double)InitialPositionGuesses.size();
			ReprojectionErrorInitial = std::sqrt(ReprojectionErrorInitial);

			double sphereInTCP[3] = { SphereCenterInTCPCoordinates.x, SphereCenterInTCPCoordinates.y, SphereCenterInTCPCoordinates.z };
			double RobotScannerTranslation[3] = { InitialTransformation.getTranslation().x(), InitialTransformation.getTranslation().y(), InitialTransformation.getTranslation().z() };
			double RobotScannerRotationQuaternion[4] = { InitialTransformationRotationQuaternion.w(), InitialTransformationRotationQuaternion.x(), InitialTransformationRotationQuaternion.y(), InitialTransformationRotationQuaternion.z() };

			ceres::Problem problem;

			problem.AddParameterBlock(RobotScannerRotationQuaternion, 4, new ceres::QuaternionParameterization());

			for (std::size_t i = 0; i < TCPToRobotSpaceTransformations.size(); i++) {
				ceres::CostFunction* cost_function = RobotScannerTCPFunctor::Create(RecognizedSphereCentersInScannerCoordinates[i], TCPToRobotSpaceTransformations[i].inv());
				problem.AddResidualBlock(cost_function,
					NULL /* squared loss */,
					sphereInTCP, RobotScannerTranslation, RobotScannerRotationQuaternion);
			}
			// Run the solver!
			ceres::Solver::Options options;
			options.minimizer_progress_to_stdout = true;
			ceres::Solver::Summary summary;
			ceres::Solve(options, &problem, &summary);
			std::cout << summary.BriefReport() << std::endl;

			SphereCenterInTCPCoordinates.x = sphereInTCP[0];
			SphereCenterInTCPCoordinates.y = sphereInTCP[1];
			SphereCenterInTCPCoordinates.z = sphereInTCP[2];

			double Length = std::sqrt(std::pow(RobotScannerRotationQuaternion[0], 2.0) + std::pow(RobotScannerRotationQuaternion[1], 2.0) + std::pow(RobotScannerRotationQuaternion[2], 2.0) + std::pow(RobotScannerRotationQuaternion[3], 2.0));
			RobotScannerRotationQuaternion[0] /= Length;
			RobotScannerRotationQuaternion[1] /= Length;
			RobotScannerRotationQuaternion[2] /= Length;
			RobotScannerRotationQuaternion[3] /= Length;

			pho::Quaternion64 FinalRotation(RobotScannerRotationQuaternion[0], RobotScannerRotationQuaternion[1], RobotScannerRotationQuaternion[2], RobotScannerRotationQuaternion[3]);
			pho::Translation3D64 FinalTranslation(RobotScannerTranslation[0], RobotScannerTranslation[1], RobotScannerTranslation[2]);

			FinalScannerToRobotTransformation = pho::Transformation3D64(FinalRotation, FinalTranslation);

			double ReprojectionErrorFinal = 0.0;
			for (std::size_t i = 0; i < InitialPositionGuesses.size(); i++) {
				ReprojectionErrorFinal += std::pow(cv::norm(FinalScannerToRobotTransformation.transformPoint(RecognizedSphereCentersInScannerCoordinates[i]) - TCPToRobotSpaceTransformations[i].transformPoint(SphereCenterInTCPCoordinates)), 2);
			}
			ReprojectionErrorFinal *= (1.0) / (double)InitialPositionGuesses.size();
			ReprojectionErrorFinal = std::sqrt(ReprojectionErrorFinal);

			std::cout << phoc("Initial ReprojectionError: #, Final Reprojection Error: #", ReprojectionErrorInitial, ReprojectionErrorFinal) << std::endl;
			return ReprojectionErrorFinal;
		}
#endif

	}


	namespace RobotCalibration {
		class SolverIterationCallback : public ceres::IterationCallback {
			std::function<ceres::CallbackReturnType(const ceres::IterationSummary& summary)> CustomCallback;
		public:
			virtual ceres::CallbackReturnType operator()(const
				ceres::IterationSummary& summary) {
				if (CustomCallback) return CustomCallback(summary);
				else return ceres::CallbackReturnType();
			}
			SolverIterationCallback(std::function<ceres::CallbackReturnType(const ceres::IterationSummary& summary)> CustomCallback) : CustomCallback(CustomCallback) {

			}

		};




		struct RobotScannerTCPFunctor {
			template <typename T> bool operator()(const T* const sphereInTCP, const T* const RobotScannerTranslation, const T* const RobotScannerRotationQuaternion, T* residual) const {
				T RobotTransformationRotationQuaternionT[4] = { (T)RobotTransformationRotationQuaternion[0], (T)RobotTransformationRotationQuaternion[1], (T)RobotTransformationRotationQuaternion[2], (T)RobotTransformationRotationQuaternion[3] };
				T Result[3] = { (T)ScannerSphereCenter[0], (T)ScannerSphereCenter[1], (T)ScannerSphereCenter[2] };
				T Temp[3];
				//ceres::UnitQuaternionRotatePoint(RobotScannerRotationQuaternion, Result, Temp);
				ceres::QuaternionRotatePoint(RobotScannerRotationQuaternion, Result, Temp);

				Temp[0] += (T)RobotScannerTranslation[0];
				Temp[1] += (T)RobotScannerTranslation[1];
				Temp[2] += (T)RobotScannerTranslation[2];

				ceres::UnitQuaternionRotatePoint(RobotTransformationRotationQuaternionT, Temp, Result);
				Result[0] += (T)RobotTransfotmationTranslation[0];
				Result[1] += (T)RobotTransfotmationTranslation[1];
				Result[2] += (T)RobotTransfotmationTranslation[2];

				residual[0] = Result[0] - sphereInTCP[0];
				residual[1] = Result[1] - sphereInTCP[1];
				residual[2] = Result[2] - sphereInTCP[2];

				/*Result[0] -= sphereInTCP[0];
				Result[1] -= sphereInTCP[1];
				Result[2] -= sphereInTCP[2];

				T Distance = ceres::sqrt(Result[0] * Result[0] + Result[1] * Result[1] + Result[2] * Result[2]);

				residual[0] = Distance;*/

				return true;
			}
			double ScannerSphereCenter[3];
			double RobotTransfotmationTranslation[3];
			double RobotTransformationRotationQuaternion[4];
			RobotScannerTCPFunctor(cv::Point3d RecognizedSphereCenter, pho::Transformation3D64 RobotTransformation) {
				ScannerSphereCenter[0] = RecognizedSphereCenter.x;
				ScannerSphereCenter[1] = RecognizedSphereCenter.y;
				ScannerSphereCenter[2] = RecognizedSphereCenter.z;

				RobotTransfotmationTranslation[0] = RobotTransformation.getTranslation().x();
				RobotTransfotmationTranslation[1] = RobotTransformation.getTranslation().y();
				RobotTransfotmationTranslation[2] = RobotTransformation.getTranslation().z();

				pho::Quaternion64 RobotTransformationQuaternionTemp(RobotTransformation.getRotation());

				RobotTransformationRotationQuaternion[0] = RobotTransformationQuaternionTemp.w();
				RobotTransformationRotationQuaternion[1] = RobotTransformationQuaternionTemp.x();
				RobotTransformationRotationQuaternion[2] = RobotTransformationQuaternionTemp.y();
				RobotTransformationRotationQuaternion[3] = RobotTransformationQuaternionTemp.z();
			}

			static ceres::CostFunction* Create(cv::Point3d RecognizedSphereCenter, pho::Transformation3D64 RobotTransformation) {
				return (new ceres::AutoDiffCostFunction<RobotScannerTCPFunctor, 3, 3, 3, 4>(new RobotScannerTCPFunctor(RecognizedSphereCenter, RobotTransformation)));
			}
		};

		double FindRobotScannerTransformation(const std::vector<pho::Transformation3D64>& TCPToRobotSpaceTransformations, const std::vector<cv::Point3d>& RecognizedSphereCentersInScannerCoordinates, pho::Transformation3D64& FinalScannerToRobotTransformation, cv::Point3d& SphereCenterInTCPCoordinates) {

			if (TCPToRobotSpaceTransformations.size() != RecognizedSphereCentersInScannerCoordinates.size()) return -1.0;
			std::vector<cv::Point3d> InitialPositionGuesses(TCPToRobotSpaceTransformations.size());
			for (std::size_t i = 0; i < InitialPositionGuesses.size(); i++) {
				InitialPositionGuesses[i] = TCPToRobotSpaceTransformations[i].transformPoint(SphereCenterInTCPCoordinates);
			}

			pho::Transformation3D64 InitialTransformation = pho::getTransformation3D_mine(RecognizedSphereCentersInScannerCoordinates, InitialPositionGuesses);
			pho::Quaternion64 InitialTransformationRotationQuaternion = InitialTransformation.getRotation();

			double ReprojectionErrorInitial = 0.0;
			for (std::size_t i = 0; i < RecognizedSphereCentersInScannerCoordinates.size(); i++) {
				ReprojectionErrorInitial += std::pow(cv::norm(InitialTransformation.transformPoint(RecognizedSphereCentersInScannerCoordinates[i]) - InitialPositionGuesses[i]), 2.0);;
			}
			ReprojectionErrorInitial *= (1.0) / (double)InitialPositionGuesses.size();
			ReprojectionErrorInitial = std::sqrt(ReprojectionErrorInitial);

			double sphereInTCP[3] = { SphereCenterInTCPCoordinates.x, SphereCenterInTCPCoordinates.y, SphereCenterInTCPCoordinates.z };
			double RobotScannerTranslation[3] = { InitialTransformation.getTranslation().x(), InitialTransformation.getTranslation().y(), InitialTransformation.getTranslation().z() };
			double RobotScannerRotationQuaternion[4] = { InitialTransformationRotationQuaternion.w(), InitialTransformationRotationQuaternion.x(), InitialTransformationRotationQuaternion.y(), InitialTransformationRotationQuaternion.z() };

			ceres::Problem problem;

			problem.AddParameterBlock(RobotScannerRotationQuaternion, 4, new ceres::QuaternionParameterization());

			for (std::size_t i = 0; i < TCPToRobotSpaceTransformations.size(); i++) {
				ceres::CostFunction* cost_function = RobotScannerTCPFunctor::Create(RecognizedSphereCentersInScannerCoordinates[i], TCPToRobotSpaceTransformations[i].inv());
				problem.AddResidualBlock(cost_function,
					NULL /* squared loss */,
					sphereInTCP, RobotScannerTranslation, RobotScannerRotationQuaternion);
			}
			// Run the solver!
			ceres::Solver::Options options;
			options.minimizer_progress_to_stdout = true;
			ceres::Solver::Summary summary;
			ceres::Solve(options, &problem, &summary);
			std::cout << summary.BriefReport() << std::endl;

			SphereCenterInTCPCoordinates.x = sphereInTCP[0];
			SphereCenterInTCPCoordinates.y = sphereInTCP[1];
			SphereCenterInTCPCoordinates.z = sphereInTCP[2];

			double Length = std::sqrt(std::pow(RobotScannerRotationQuaternion[0], 2.0) + std::pow(RobotScannerRotationQuaternion[1], 2.0) + std::pow(RobotScannerRotationQuaternion[2], 2.0) + std::pow(RobotScannerRotationQuaternion[3], 2.0));
			RobotScannerRotationQuaternion[0] /= Length;
			RobotScannerRotationQuaternion[1] /= Length;
			RobotScannerRotationQuaternion[2] /= Length;
			RobotScannerRotationQuaternion[3] /= Length;

			pho::Quaternion64 FinalRotation(RobotScannerRotationQuaternion[0], RobotScannerRotationQuaternion[1], RobotScannerRotationQuaternion[2], RobotScannerRotationQuaternion[3]);
			pho::Translation3D64 FinalTranslation(RobotScannerTranslation[0], RobotScannerTranslation[1], RobotScannerTranslation[2]);

			FinalScannerToRobotTransformation = pho::Transformation3D64(FinalRotation, FinalTranslation);

			double ReprojectionErrorFinal = 0.0;
			for (std::size_t i = 0; i < InitialPositionGuesses.size(); i++) {
				ReprojectionErrorFinal += std::pow(cv::norm(FinalScannerToRobotTransformation.transformPoint(RecognizedSphereCentersInScannerCoordinates[i]) - TCPToRobotSpaceTransformations[i].transformPoint(SphereCenterInTCPCoordinates)), 2);
			}
			ReprojectionErrorFinal *= (1.0) / (double)InitialPositionGuesses.size();
			ReprojectionErrorFinal = std::sqrt(ReprojectionErrorFinal);

			std::cout << phoc("Initial ReprojectionError: #, Final Reprojection Error: #", ReprojectionErrorInitial, ReprojectionErrorFinal) << std::endl;
			return ReprojectionErrorFinal;
		}

		void DoCalibrationTestFunction(const std::vector<pho::Transformation3D64>& TCPToRobotSpaceTransformations, const std::vector<cv::Point3d>& RecognizedSphereCentersInScannerCoordinates) {
			if (TCPToRobotSpaceTransformations.size() != RecognizedSphereCentersInScannerCoordinates.size()) return;
			cv::Point3d SphereCenterInTCPCoordinates(0.0, 0.0, 0.0);
			pho::Transformation3D64 FinalScannerToRobotTransformation;

			std::vector<cv::Point3d> RobotCoordinates(TCPToRobotSpaceTransformations.size());
			pho::Transformation3D64 FinalRobotToScannerTransformation;

			const int Order = 1;
			double Offset = 100.0;

			double XMin = RecognizedSphereCentersInScannerCoordinates[0].x;
			double YMin = RecognizedSphereCentersInScannerCoordinates[0].y;
			double ZMin = RecognizedSphereCentersInScannerCoordinates[0].z;
			double XMax = RecognizedSphereCentersInScannerCoordinates[0].x;
			double YMax = RecognizedSphereCentersInScannerCoordinates[0].y;
			double ZMax = RecognizedSphereCentersInScannerCoordinates[0].z;
			for (std::size_t i = 0; i < RecognizedSphereCentersInScannerCoordinates.size(); i++) {
				XMin = std::min(XMin, RecognizedSphereCentersInScannerCoordinates[i].x);
				YMin = std::min(YMin, RecognizedSphereCentersInScannerCoordinates[i].y);
				ZMin = std::min(ZMin, RecognizedSphereCentersInScannerCoordinates[i].z);
				XMax = std::max(XMax, RecognizedSphereCentersInScannerCoordinates[i].x);
				YMax = std::max(YMax, RecognizedSphereCentersInScannerCoordinates[i].y);
				ZMax = std::max(ZMax, RecognizedSphereCentersInScannerCoordinates[i].z);
			}
			XMin -= Offset;
			YMin -= Offset;
			ZMin -= Offset;
			XMax += Offset;
			YMax += Offset;
			ZMax += Offset;

			std::vector<cv::Point3d> RecognizedSphereCentersInScannerCoordinatesLocal = RecognizedSphereCentersInScannerCoordinates;
			int Iterations = 3;
			for (int i = 0; i < Iterations; i++) {

				FindRobotScannerTransformation(TCPToRobotSpaceTransformations, RecognizedSphereCentersInScannerCoordinatesLocal, FinalScannerToRobotTransformation, SphereCenterInTCPCoordinates);
				FinalRobotToScannerTransformation = FinalScannerToRobotTransformation.inv();


				for (std::size_t i = 0; i < RobotCoordinates.size(); i++) {
					RobotCoordinates[i] = FinalRobotToScannerTransformation.transformPoint(TCPToRobotSpaceTransformations[i].transformPoint(SphereCenterInTCPCoordinates));
				}
				pho::Volumetric::BezierVolumeWarperPrism64 BezierVolumePrism(XMin, YMin, ZMin, XMax, YMax, ZMax);
				double InitialReprojectionError, FinalReprojectionError, InitialReprojectionErrorMSQR, FinalReprojectionErrorMSQR;

				double FitDuration, CreateAccelaratedStructureDuration, WarpBezier, WarpAccelerated;
				CDuration Duration;

				Duration.Start();
				BezierVolumePrism.FitVolume(RobotCoordinates, RecognizedSphereCentersInScannerCoordinates, Order, InitialReprojectionError, FinalReprojectionError, InitialReprojectionErrorMSQR, FinalReprojectionErrorMSQR);
				Duration.Stop(); FitDuration = Duration.GetDuration();

				phoc << phoc.ToString("Initial reprojection error: # , MSQR # --> Final reprojection error: #, MSQR #", InitialReprojectionError, InitialReprojectionErrorMSQR, FinalReprojectionError, FinalReprojectionErrorMSQR) << std::endl;

				for (std::size_t i = 0; i < RecognizedSphereCentersInScannerCoordinatesLocal.size(); i++) {
					RecognizedSphereCentersInScannerCoordinatesLocal[i] = BezierVolumePrism.WarpPoint(RecognizedSphereCentersInScannerCoordinates[i]);
				}
			}
		}

		void DoCalibrationTestFunction(const std::vector<pho::Transformation3D64>& TCPToRobotSpaceTransformations, const std::vector<std::vector<cv::Point3d>>& RecognizedSpheresPoints, double& SphereRadius, bool OptimizeIndividualSpherePoints = true) {
			if (TCPToRobotSpaceTransformations.size() != RecognizedSpheresPoints.size()) return;
			cv::Point3d SphereCenterInTCPCoordinates(0.0, 0.0, 0.0);
			pho::Transformation3D64 FinalScannerToRobotTransformation;

			std::vector<cv::Point3d> RobotCoordinates(TCPToRobotSpaceTransformations.size());
			pho::Transformation3D64 FinalRobotToScannerTransformation;


			std::vector<cv::Point3d> RecognizedSphereCentersInScannerCoordinates(RecognizedSpheresPoints.size(), cv::Point3d(0.0, 0.0, 0.0));
			for (std::size_t i = 0; i < RecognizedSpheresPoints.size(); i++) {
				if (RecognizedSpheresPoints[i].empty()) return;
				for (std::size_t k = 0; k < RecognizedSpheresPoints[i].size(); k++) {
					RecognizedSphereCentersInScannerCoordinates[i] += RecognizedSpheresPoints[i][k];
				}
				RecognizedSphereCentersInScannerCoordinates[i] *= 1.0 / (double)RecognizedSpheresPoints[i].size();
			}

			const int Order = 3;
			double Offset = 100.0;

			double XMin = RecognizedSphereCentersInScannerCoordinates[0].x;
			double YMin = RecognizedSphereCentersInScannerCoordinates[0].y;
			double ZMin = RecognizedSphereCentersInScannerCoordinates[0].z;
			double XMax = RecognizedSphereCentersInScannerCoordinates[0].x;
			double YMax = RecognizedSphereCentersInScannerCoordinates[0].y;
			double ZMax = RecognizedSphereCentersInScannerCoordinates[0].z;
			for (std::size_t i = 0; i < RecognizedSphereCentersInScannerCoordinates.size(); i++) {
				XMin = std::min(XMin, RecognizedSphereCentersInScannerCoordinates[i].x);
				YMin = std::min(YMin, RecognizedSphereCentersInScannerCoordinates[i].y);
				ZMin = std::min(ZMin, RecognizedSphereCentersInScannerCoordinates[i].z);
				XMax = std::max(XMax, RecognizedSphereCentersInScannerCoordinates[i].x);
				YMax = std::max(YMax, RecognizedSphereCentersInScannerCoordinates[i].y);
				ZMax = std::max(ZMax, RecognizedSphereCentersInScannerCoordinates[i].z);
			}
			XMin -= Offset;
			YMin -= Offset;
			ZMin -= Offset;
			XMax += Offset;
			YMax += Offset;
			ZMax += Offset;


			pho::Volumetric::BezierVolumeWarperPrism64 BezierVolumePrism(XMin, YMin, ZMin, XMax, YMax, ZMax);
			std::vector<cv::Point3d> RecognizedSphereCentersInScannerCoordinatesLocal = RecognizedSphereCentersInScannerCoordinates;
			std::vector<std::vector<cv::Point3d>> RecognizedSpherePointsLocal = RecognizedSpheresPoints;


			pho::CeresSphereFitter::FitSpheresWithRadius(RecognizedSpherePointsLocal, RecognizedSphereCentersInScannerCoordinatesLocal, SphereRadius);
			phoc("Current SphereRadius: #", SphereRadius);
			RecognizedSphereCentersInScannerCoordinates = RecognizedSphereCentersInScannerCoordinatesLocal;


			int Iterations = 3;
			for (int i = 0; i < Iterations; i++) {

				if (i != 0 && OptimizeIndividualSpherePoints) {
					pho::CeresSphereFitter::FitSpheresWithRadius(RecognizedSpherePointsLocal, RecognizedSphereCentersInScannerCoordinatesLocal, SphereRadius);
					phoc("Current SphereRadius: #", SphereRadius);
				}

				FindRobotScannerTransformation(TCPToRobotSpaceTransformations, RecognizedSphereCentersInScannerCoordinatesLocal, FinalScannerToRobotTransformation, SphereCenterInTCPCoordinates);
				FinalRobotToScannerTransformation = FinalScannerToRobotTransformation.inv();

				for (std::size_t i = 0; i < RobotCoordinates.size(); i++) {
					if (OptimizeIndividualSpherePoints) {
						if (BezierVolumePrism.isBezierVolumePresent()) {
							RecognizedSphereCentersInScannerCoordinates[i] = BezierVolumePrism.UnWarpPoint(RecognizedSphereCentersInScannerCoordinatesLocal[i]);
						}
						else {
							RecognizedSphereCentersInScannerCoordinates[i] = RecognizedSphereCentersInScannerCoordinatesLocal[i];
						}
					}
					RobotCoordinates[i] = FinalRobotToScannerTransformation.transformPoint(TCPToRobotSpaceTransformations[i].transformPoint(SphereCenterInTCPCoordinates));
				}

				double InitialReprojectionError, FinalReprojectionError, InitialReprojectionErrorMSQR, FinalReprojectionErrorMSQR;

				double FitDuration, CreateAccelaratedStructureDuration, WarpBezier, WarpAccelerated;
				CDuration Duration;

				Duration.Start();
				BezierVolumePrism.FitVolume(RobotCoordinates, RecognizedSphereCentersInScannerCoordinates, Order, InitialReprojectionError, FinalReprojectionError, InitialReprojectionErrorMSQR, FinalReprojectionErrorMSQR);
				Duration.Stop(); FitDuration = Duration.GetDuration();

				phoc << phoc.ToString("Initial reprojection error: # , MSQR # --> Final reprojection error: #, MSQR #", InitialReprojectionError, InitialReprojectionErrorMSQR, FinalReprojectionError, FinalReprojectionErrorMSQR) << std::endl;

				if (OptimizeIndividualSpherePoints) {
					for (std::size_t i = 0; i < RecognizedSpherePointsLocal.size(); i++) {
						for (std::size_t k = 0; k < RecognizedSpherePointsLocal[i].size(); k++) {
							RecognizedSpherePointsLocal[i][k] = BezierVolumePrism.WarpPoint(RecognizedSpheresPoints[i][k]);
						}
					}
				}
				else {
					for (std::size_t i = 0; i < RecognizedSphereCentersInScannerCoordinatesLocal.size(); i++) {
						RecognizedSphereCentersInScannerCoordinatesLocal[i] = BezierVolumePrism.WarpPoint(RecognizedSphereCentersInScannerCoordinates[i]);
					}
				}
			}

			pho::Volumetric::BezierVolumeTransformBox TransferBox = BezierVolumePrism;
			pho::api::PhoXiFactory Factory;
			pho::api::PPhoXi Scanner = Factory.CreateAndConnectFirstAttached();
			if (Scanner) {
				pho::api::PhoXiRawAccessHandler RawHandler(Scanner);
				pho::DataManager InputDM, InputDM2;
				InputDM["RobotSpaceVolumeWarp.BezierVolumeTransformBox"] = TransferBox;
				pho::DataManager OutputDM;
				/*cv::FileStorage fs1("Output/TestTransferBox.xml", cv::FileStorage::WRITE);
				fs1 << "TestDM";
				InputDM.Write(fs1, XMLCompatible::WriteType::FullWrite);
				fs1.release();

				cv::FileStorage fs2("Output/TestTransferBox.xml", cv::FileStorage::READ);
				InputDM2.Read(fs2.getFirstTopLevelNode());
				fs2.release();

				cv::FileStorage fs3("Output/TestTransferBox2.xml", cv::FileStorage::WRITE);
				fs3 << "TestDM";
				InputDM2.Write(fs3, XMLCompatible::WriteType::FullWrite);
				fs3.release();*/


				phoc << (std::string)RawHandler.AdditionalCommand("SetRobotSpaceVolumeWarp", InputDM, OutputDM) << std::endl;
			}

		}
	}
}

void RecodeTest() {
	cv::Mat Eye = cv::Mat::eye(cv::Size(1000, 1000), CV_8UC1);
	Eye *= 255;
	cv::imshow("Mat", Eye);
	cv::waitKey(0);
}


class SimpleNormalsEstimator {
private:
	const cv::Point3f ZeroPointF;
	cv::Mat PointCloud;
	int Radius;
	double Radius3D;
	void InitiateOutput() {
		for (int i = 0; i < 5; ++i) {
			Outputs[i] = cv::Mat::zeros(PointCloud.size(), CV_32FC3);
		}
	}
	void SetZero(int y, int x) {
		for (int i = 0; i < 5; ++i) {
			Outputs[i].at<cv::Point3f>(y, x) = ZeroPointF;
		}
	}
	inline cv::Point3d normalFromMatrix(double m[][3], double& eig1, double& eig2, double& eig3) {
		//https://en.wikipedia.org/wiki/Eigenvalue_algorithm#3.C3.973_matrices
		//float eig1, eig2, eig3;
		double p1 = m[0][1] * m[0][1] + m[0][2] * m[0][2] + m[1][2] * m[1][2];
		if (fabs(p1) < 0.01) {
			eig1 = m[0][0];
			eig2 = m[1][1];
			eig3 = m[2][2];
		}
		else {
			double trace = m[0][0] + m[1][1] + m[2][2];
			double q = trace / 3.0f;
			double p2 = (m[0][0] - q) * (m[0][0] - q) + (m[1][1] - q) * (m[1][1] - q) + (m[2][2] - q) * (m[2][2] - q) + 2.0 * p1;
			double p = sqrt(p2 / 6.0);
			//                            B = (1 / p) * (A - q * I)
			double b[3][3];
			//                            multiplyMatrixNumber(identity, q, temp);
			//                            subtractMatrix(m, temp, temp2);
			//                            multiplyMatrixNumber(temp2, 1 / p, b);
			for (int i = 0; i < 3; i++) {
				for (int j = 0; j < 3; j++) {
					if (i == j) b[i][j] = (m[i][j] - q) / p;
					else b[i][j] = m[i][j] / p;
				}
			}
			//                b[0][0] * (b[1][1] * b[2][2] - b[1][2] * b[2][1]) - b[0][1] * (b[1][0] * b[2][2] - b[1][2] * b[2][0]) + b[0][2] * (b[1][0] * b[2][1] - b[1][1] * b[2][0]);

			double detb = b[0][0] * ((b[1][1] * b[2][2]) - (b[2][1] * b[1][2])) - b[0][1] * (b[1][0] * b[2][2] - b[2][0] * b[1][2]) + b[0][2] * (b[1][0] * b[2][1] - b[2][0] * b[1][1]);
			double r = detb / 2.0;

			double phi;

			if (r <= -1.0) {
				phi = CV_PI / 3.0;
			}
			else if (r >= 1.0) {
				phi = 0;
			}
			else {
				phi = acos(r) / 3.0;
			}
			// the eigenvalues satisfy eig3 <= eig2 <= eig1
			eig1 = q + 2.0 * p * cos(phi);
			eig3 = q + 2.0 * p * cos(phi + (2.0 * CV_PI / 3.0));
			eig2 = 3.0 * q - eig1 - eig3;
		}

		double out[3][3];

		cv::Point3d normal = cv::Point3d(0.0, 0.0, 0.0);
		double fir, sec;
		for (int col = 0; col < 3; col++) {
			for (int row = 0; row < 3; row++) {
				out[row][col] = 0;

				fir = m[row][0], sec = m[0][col];
				if (row == 0) fir -= eig1;
				if (0 == col) sec -= eig2;
				out[row][col] += fir * sec;

				fir = m[row][1], sec = m[1][col];
				if (row == 1) fir -= eig1;
				if (1 == col) sec -= eig2;
				out[row][col] += fir * sec;

				fir = m[row][2], sec = m[2][col];
				if (row == 2) fir -= eig1;
				if (2 == col) sec -= eig2;
				out[row][col] += fir * sec;
			}
			if (out[0][col] != 0.0f || out[1][col] != 0.0f || out[2][col] != 0.0f) {
				normal = cv::Point3d(out[0][col], out[1][col], out[2][col]);
			}
		}
		if (normal.z > 0.0) normal *= -1.0;
		return normal;
	}
public:
	cv::Mat Outputs[5];
	cv::Mat& NorthNormals;
	cv::Mat& EastNormals;
	cv::Mat& SouthNormals;
	cv::Mat& WestNormals;
	cv::Mat& FullNormals;
	SimpleNormalsEstimator() : ZeroPointF(cv::Point3f(0.0, 0.0, 0.0)),
		NorthNormals(Outputs[0]), EastNormals(Outputs[1]), SouthNormals(Outputs[2]), WestNormals(Outputs[3]), FullNormals(Outputs[4]) {};



	void ComputeNormalPoint(const cv::Mat& DiffPoints, cv::Point3f& Output) {
		const int minimalPatchPoints = 1;
		double complete_sum_xx = 0.0, complete_sum_yy = 0.0, complete_sum_zz = 0.0, complete_sum_xy = 0.0, complete_sum_xz = 0.0, complete_sum_yz = 0.0, complete_sum_x = 0.0, complete_sum_y = 0.0, complete_sum_z = 0.0;
		double eig1;
		double eig2;
		double eig3;
		int CorrectPoints = 0;

		for (int y = 0; y < DiffPoints.rows; ++y) {
			for (int x = 0; x < DiffPoints.cols; ++x) {
				cv::Point3d InspectedPoint = DiffPoints.at<cv::Point3d>(y, x);
				if (InspectedPoint.z < Radius3D) {
					complete_sum_xx += InspectedPoint.x * InspectedPoint.x;
					complete_sum_yy += InspectedPoint.y * InspectedPoint.y;
					complete_sum_zz += InspectedPoint.z * InspectedPoint.z;
					complete_sum_xy += InspectedPoint.x * InspectedPoint.y;
					complete_sum_xz += InspectedPoint.x * InspectedPoint.z;
					complete_sum_yz += InspectedPoint.y * InspectedPoint.z;
					complete_sum_x += InspectedPoint.x;
					complete_sum_y += InspectedPoint.y;
					complete_sum_z += InspectedPoint.z;
					++CorrectPoints;
				}
			}
		}
		double m[3][3];
		if (CorrectPoints > minimalPatchPoints) {
			double scale = 1.0 / (double)CorrectPoints;
			m[0][1] = m[1][0] = (double)(complete_sum_xy - (complete_sum_x * (complete_sum_y * scale)));
			m[0][0] = (double)(complete_sum_xx - (complete_sum_x * (complete_sum_x * scale)));
			m[0][2] = m[2][0] = (double)(complete_sum_xz - (complete_sum_x * (complete_sum_z * scale)));
			m[1][1] = (double)(complete_sum_yy - (complete_sum_y * (complete_sum_y * scale)));
			m[1][2] = m[2][1] = (double)(complete_sum_yz - (complete_sum_y * (complete_sum_z * scale)));
			m[2][2] = (double)(complete_sum_zz - (complete_sum_z * (complete_sum_z * scale)));
		}
		else {
			m[0][0] = m[0][1] = m[0][2] = m[1][0] = m[1][1] = m[1][2] = m[2][0] = m[2][1] = m[2][2] = 0.0f;
		}
		Output = (cv::Point3f)normalFromMatrix(m, eig1, eig2, eig3);
	}

	void ComputeNormalPoints(int y, int x, const cv::Mat& CenterDiffPoints) {
		ComputeNormalPoint(CenterDiffPoints(cv::Rect(0, 0, Radius * 2 + 1, Radius + 1)), NorthNormals.at<cv::Point3f>(y, x));
		ComputeNormalPoint(CenterDiffPoints(cv::Rect(0, Radius, Radius * 2 + 1, Radius + 1)), SouthNormals.at<cv::Point3f>(y, x));
		ComputeNormalPoint(CenterDiffPoints(cv::Rect(0, 0, Radius + 1, Radius * 2 + 1)), WestNormals.at<cv::Point3f>(y, x));
		ComputeNormalPoint(CenterDiffPoints(cv::Rect(Radius, 0, Radius + 1, Radius * 2 + 1)), EastNormals.at<cv::Point3f>(y, x));
		ComputeNormalPoint(CenterDiffPoints, FullNormals.at<cv::Point3f>(y, x));
	}
	void EstimateNormals(const cv::Mat& PointCloud, int Radius = 2, double PointDistanceAt1000MM = 0.44342430545947999739335482999474) {
		this->PointCloud = PointCloud.clone();
		this->Radius = Radius;
		Radius3D = (PointDistanceAt1000MM * (float)Radius) * 3.0f * sqrt(2.0f);
		double Radius3DSquared = Radius3D * Radius3D;
		InitiateOutput();
		cv::Mat CenterDiffPoints(cv::Size(2 * Radius + 1, 2 * Radius + 1), CV_64FC3, cv::Scalar(0.0, 0.0, 0.0));
		cv::Point3d ZeroPoint = cv::Point3d(0.0, 0.0, 0.0);
		cv::Point3d NULLPoint = cv::Point3d(2.0 * Radius3D, 2.0 * Radius3D, 2.0 * Radius3D);
		for (int y = Radius; y < PointCloud.rows - Radius - 1; ++y) {
			for (int x = Radius; x < PointCloud.cols - Radius - 1; ++x) {
				cv::Point3d ActualPoint = (cv::Point3d)PointCloud.at<cv::Point3f>(y, x);
				if (ActualPoint == ZeroPoint) {
					SetZero(y, x);
					continue;
				}
				//Pozor, toto predpoklada ze je suradnicova sustava z 0 v kamere
				double Threshold = Radius3DSquared * std::pow(ActualPoint.z / 1000.0, 2.0);
				for (int yy = y - Radius, yyy = 0; yy <= y + Radius; ++yy, ++yyy) {
					for (int xx = x - Radius, xxx = 0; xx <= x + Radius; ++xx, ++xxx) {
						cv::Point3d ActualPointLocal = (cv::Point3d)PointCloud.at<cv::Point3f>(yy, xx);
						if (ActualPointLocal == ZeroPoint) {
							CenterDiffPoints.at<cv::Point3d>(yyy, xxx) = NULLPoint;
							continue;
						}
						else {
							cv::Point3d Difference = ActualPointLocal - ActualPoint;
							if (Difference.dot(Difference) < Threshold) {
								CenterDiffPoints.at<cv::Point3d>(yyy, xxx) = Difference;
							}
							else {
								CenterDiffPoints.at<cv::Point3d>(yyy, xxx) = NULLPoint;
							}
						}
					}
				}
				ComputeNormalPoints(y, x, CenterDiffPoints);
			}
		}
	}

	static void HowToUse() {
		pho::PointCloud Cloud = pho::Plyio::plyRead("Output/TestCloud.ply");

		cv::Mat PointCloud = Cloud.points;
		cv::Mat Normals = Cloud.normals;
		cv::Mat Colors = Cloud.colors;

		SimpleNormalsEstimator NormalsEstimator;
		NormalsEstimator.EstimateNormals(PointCloud);

		pho::PointCloud North(PointCloud, Colors, NormalsEstimator.NorthNormals);
		pho::PointCloud East(PointCloud, Colors, NormalsEstimator.EastNormals);
		pho::PointCloud South(PointCloud, Colors, NormalsEstimator.SouthNormals);
		pho::PointCloud West(PointCloud, Colors, NormalsEstimator.WestNormals);
		pho::PointCloud Full(PointCloud, Colors, NormalsEstimator.FullNormals);

		pho::Plyio::plyWrite("Output/NormlasNorth.ply", North);
		pho::Plyio::plyWrite("Output/NormlasEast.ply", East);
		pho::Plyio::plyWrite("Output/NormlasSouth.ply", South);
		pho::Plyio::plyWrite("Output/NormlasWest.ply", West);
		pho::Plyio::plyWrite("Output/NormlasFull.ply", Full);
	}
};

class ThorlabsAxisCalibration {
public:
	std::vector<std::pair<double, double>> CalibrationEntries;
	ThorlabsAxisCalibration(const std::vector<std::pair<double, double>>& CalibrationEntries) : CalibrationEntries(CalibrationEntries) {
	}
	double GetCalibratedPosition(double Position) {
		std::size_t Index = 0;
		if (CalibrationEntries.size() <= 2) return -10000000.0;
		for (; Index < CalibrationEntries.size(); ++Index) {
			if (Index == CalibrationEntries.size() - 1) return -10000000.0;
			if (CalibrationEntries[Index].first <= Position && CalibrationEntries[Index + 1].first > Position) {
				break;
			}
		}
		double EntryDistance = CalibrationEntries[Index + 1].first - CalibrationEntries[Index].first;
		double DistanceFromSmaller = Position - CalibrationEntries[Index].first;
		double CorrectPoistion = ((EntryDistance - DistanceFromSmaller) * CalibrationEntries[Index].second + DistanceFromSmaller * CalibrationEntries[Index + 1].second) / EntryDistance;
		return CorrectPoistion;
	}
};


#include "LinearAxis/ThorlabsKinesisLinearAxis.h"

void ThorlabsAxisTest() {

	pho::ThorlabsKinesisLinearAxis Axis("", "C:\\Users\\tomas\\Desktop\\ThorlabsKalibraciaOsi\\45871809.dat");

	if (Axis.isInitiated()) {
		std::cout << Axis.GetPosition() << std::endl;
		if (Axis.NeedsHoming()) Axis.GoHome();
		Axis.GoToPosition(50.0, true);
		std::cout << Axis.GetPosition() << std::endl;
		Axis.GoToPosition(150.0, true);
		std::cout << Axis.GetPosition() << std::endl;
		Axis.GoToPosition(250.0, true);
		std::cout << Axis.GetPosition() << std::endl;
		Axis.GoToPosition(100.0, true);
		std::cout << Axis.GetPosition() << std::endl;
	}

	return;

	std::string SerialNumber;

	int serialNo = 45837825;

	// get parameters from command line        
	int position = 0;
	int velocity = 0;
	// identify and access device        
	char testSerialNo[16];
	sprintf_s(testSerialNo, "%d", serialNo);
	try {

		// Build list of connected device            
		if (TLI_BuildDeviceList() == 0) {

			// get device list size                 
			short n = TLI_GetDeviceListSize();

			// get LTS serial numbers                
			char serialNos[100];
			TLI_GetDeviceListByTypeExt(serialNos, 100, 45);

			// output list of matching devices                
			char *p = strtok(serialNos, ",");
			while (p != NULL) {
				TLI_DeviceInfo deviceInfo;

				// get device info from device                    
				TLI_GetDeviceInfo(p, &deviceInfo);

				// get strings from device info structure                    
				char desc[65];
				strncpy(desc, deviceInfo.description, 64);
				desc[64] = '\0';
				char serialNo[9];
				strncpy(serialNo, deviceInfo.serialNo, 8);
				serialNo[8] = '\0';

				// output                    
				printf("Found Device %s=%s : %s\r\n", p, serialNo, desc);
				p = strtok(NULL, ",");
				for (int i = 0; i < 9; ++i) {
					testSerialNo[i] = serialNo[i];
				}
			}
			// open device                
			if (ISC_Open(testSerialNo) == 0) {

				// start the device polling at 200ms intervals                    
				ISC_StartPolling(testSerialNo, 200);
				Sleep(3000);


				std::string CalibrationFilePath = "C:\\Users\\tomas\\Desktop\\ThorlabsKalibraciaOsi\\45871809.dat";

				std::vector<std::pair<double, double>> CalibrationEntries;

				{
					std::ifstream CalibrationFileStream(CalibrationFilePath);
					if (CalibrationFileStream.is_open()) {
						while (!CalibrationFileStream.eof()) {
							double Position, CorrectPosition;
							if (CalibrationFileStream.eof()) break;
							CalibrationFileStream >> Position;
							if (CalibrationFileStream.eof()) break;
							CalibrationFileStream >> CorrectPosition;
							CalibrationEntries.push_back(std::pair<double, double>(Position, CorrectPosition));
						}
						CalibrationFileStream.close();
					}
				}

				ThorlabsAxisCalibration Calibration(CalibrationEntries);

				//ISC_SetCalibrationFile(testSerialNo, CalibrationFilePath.c_str(), true);
				//bool calibrationEnabled = ISC_IsCalibrationActive(testSerialNo);
				// Home device                    
				ISC_ClearMessageQueue(testSerialNo);
				ISC_Home(testSerialNo);


				printf("Device %s homing\r\n", testSerialNo);

				// wait for completion                    
				WORD messageType;
				WORD messageId;
				DWORD messageData;
				ISC_WaitForMessage(testSerialNo, &messageType, &messageId, &messageData);
				while (messageType != 2 || messageId != 0) {
					ISC_WaitForMessage(testSerialNo, &messageType, &messageId, &messageData);
				}

				int maximalPosition = ISC_GetNumberPositions(testSerialNo);
				long currentBackslash = ISC_GetBacklash(testSerialNo);
				long currentPotisionCounter = ISC_GetPositionCounter(testSerialNo);
				int currentPosition = ISC_GetPosition(testSerialNo);

				double realPosition;

				//409600 je steps na mm
				short ReturnCode = ISC_GetRealValueFromDeviceUnit(testSerialNo, currentPosition, &realPosition, 0);
				ReturnCode = ISC_GetRealValueFromDeviceUnit(testSerialNo, currentPosition, &realPosition, 1);
				ReturnCode = ISC_GetRealValueFromDeviceUnit(testSerialNo, currentPosition, &realPosition, 2);
				ReturnCode = ISC_GetRealValueFromDeviceUnit(testSerialNo, currentPosition, &realPosition, 3);
				ReturnCode = ISC_GetRealValueFromDeviceUnit(testSerialNo, currentPosition, &realPosition, -1);

				realPosition = (double)currentPosition / 409600.0;
				double realPositionCalibrated = Calibration.GetCalibratedPosition(realPosition);

				double newPositionMM = 100.0;
				double newPositionMMRetrieved;
				int newPositionDevice;
				ReturnCode = ISC_GetDeviceUnitFromRealValue(testSerialNo, newPositionMM, &newPositionDevice, 0);
				newPositionDevice = (int)(newPositionMM * 409600.0);
				position = newPositionDevice;


				// set velocity if desired                    
				if (velocity > 0) {
					int currentVelocity, currentAcceleration;
					ISC_GetVelParams(testSerialNo, &currentVelocity, &currentAcceleration);
					ISC_SetVelParams(testSerialNo, velocity, currentAcceleration);
				}
				// move to position (channel 1)                    
				ISC_ClearMessageQueue(testSerialNo);
				ISC_MoveToPosition(testSerialNo, position);
				printf("Device %s moving\r\n", testSerialNo);

				// wait for completion                    
				ISC_WaitForMessage(testSerialNo, &messageType, &messageId, &messageData);
				while (messageType != 2 || messageId != 1) {
					ISC_WaitForMessage(testSerialNo, &messageType, &messageId, &messageData);
				}
				// get actual poaition                    
				int pos = ISC_GetPosition(testSerialNo);
				printf("Device %s moved to %d\r\n", testSerialNo, pos);

				long currentPotisionCounter2 = ISC_GetPositionCounter(testSerialNo);
				ReturnCode = ISC_GetRealValueFromDeviceUnit(testSerialNo, pos, &newPositionMMRetrieved, 0);
				newPositionMMRetrieved = (double)pos / 409600.0;

				double newPositionMMRetrievedCalibrated = Calibration.GetCalibratedPosition(newPositionMMRetrieved);

				// stop polling                    
				ISC_StopPolling(testSerialNo);

				// close device                    
				ISC_Close(testSerialNo);
			}
		}
	}
	catch (char * e) {
		printf("Error %s\r\n", e);
	}
	//char c = _getch();
	return;
}


//------------------------------------------------------------------------------------------------------------

#define SweepUpTimePath "Global/Projector/Settings/SweepUpTime.AHBox<double>"
#define SweepDownTimePath "Global/Projector/Settings/SweepDownTime.AHBox<double>"
#define IndexMapPath "Actions/BasicDepthAction/0/index.Mat"
#define TexturePath "User/Texture.Mat"
#define DepthPath "User/DepthMap.Mat"
#define PointCloudPath "User/PointCloud.Mat"
#define UseCalibratedIndexOffsetsPath "Global/Projector/Settings/UseCalibratedIndexOffsets.AHBox<bool>"

//------------------------------------------------------------------------------------------------------------

class RGBDFrame
{
public:
	UINT64 m_colorSizeBytes;					//compressed byte size
	UINT64 m_depthSizeBytes;					//compressed byte size
	unsigned char* m_colorCompressed;			//compressed color data
	unsigned char* m_depthCompressed;			//compressed depth data
	UINT64 m_timeStampColor;					//time stamp color (convection: in microseconds)
	UINT64 m_timeStampDepth;					//time stamp depth (convention: in microseconds)
	//mat4f m_cameraToWorld;						//camera trajectory: from current frame to base frame
	cv::Matx44f m_cameraToWorld;						//camera trajectory: from current frame to base frame

	cv::Mat tex;
	cv::Mat depth;

	void saveToFile(std::ofstream& out) const {
		out.write((const char*)&m_cameraToWorld, sizeof(cv::Matx44f));
		out.write((const char*)&m_timeStampColor, sizeof(UINT64));
		out.write((const char*)&m_timeStampDepth, sizeof(UINT64));
		out.write((const char*)&m_colorSizeBytes, sizeof(UINT64));
		out.write((const char*)&m_depthSizeBytes, sizeof(UINT64));
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
		UINT64 strLen = m_sensorName.size();
		out.write((const char*)&strLen, sizeof(UINT64));
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

		UINT64 numFrames = m_frames.size();
		out.write((const char*)&numFrames, sizeof(UINT64));
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

inline cv::Mat ComputeInverseCoordinateSpaceTransformation(const cv::Mat &CoordinateSpaceTransformation) {
	cv::Mat Result = cv::Mat::eye(4, 4, CV_32F);
	CoordinateSpaceTransformation(cv::Rect(0, 0, 3, 3)).copyTo(Result(cv::Rect(0, 0, 3, 3)));
	cv::transpose(Result, Result);
	cv::Mat TranslationVector =
		Result(cv::Rect(0, 0, 3, 3)) * CoordinateSpaceTransformation(cv::Rect(3, 0, 1, 3)) * (-1.0);
	TranslationVector.copyTo(Result(cv::Rect(3, 0, 1, 3)));
	return Result;
}

//------------------------------------------------------------------------------------------------------------

template<class T>
bool ReprojectToDepthMap(const photoneoTools::CameraCalibration<T>& cameraCalibration, cv::Mat& InputCloud, const cv::Size& OutputResolution, cv::Mat& OutputDepthMap, const cv::Mat& InputTexture, cv::Mat& OutputTexture, T IntegrationThreshold) {
	std::vector<cv::Point_<T>> ProjectedCoordinates;
	cv::Mat InputCloudRaw = InputCloud;
	cv::Mat InputCloudWorking = InputCloudRaw;
	if (InputCloudRaw.cols > 1 && InputCloudRaw.rows > 1) {
		InputCloudWorking = InputCloudRaw.reshape(3, InputCloudRaw.size().area());
	}

	cameraCalibration.Project3DPointsToImagePoints(InputCloudWorking, ProjectedCoordinates, false);

	cv::Size_<T> OutputResolutionInternal = (cv::Size_<T>)OutputResolution;
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

double lastTimestamp = -1;

bool Scan(pho::api::PPhoXi PhoXiDevice, int index, SensorData& sd)
{
	std::cout << "Triggering the " << "1" << "-th frame" << std::endl;
	std::string CustomMessage = "Hello World - Software Trigger Frame Number: 1";
	int FrameID = PhoXiDevice->TriggerFrame(true, false, CustomMessage);
	if (FrameID < 0) {
		//If negative number is returned trigger was unsuccessful
		std::cout << "Trigger was unsuccessful!" << std::endl;
		return false;
	}
	else {
		std::cout << "Frame was triggered, Frame Id: " << FrameID << std::endl;
	}
	std::cout << "Waiting for frame 1" << std::endl;
	pho::api::PFrame Frame = PhoXiDevice->GetSpecificFrame(FrameID, pho::api::PhoXiTimeout::Infinity);
	if (Frame) {
		std::cout << "Frame retrieved" << std::endl;
		std::cout << "    Sensor Position: " << Frame->Info.SensorPosition.x << "; " << Frame->Info.SensorPosition.y << "; " << Frame->Info.SensorPosition.z << std::endl;
		std::cout << "    Frame Custom Message: " << Frame->CustomMessage << std::endl;


		pho::api::PhoXiRawAccessHandler RawAccessHandler(PhoXiDevice);

		pho::PDataManager WholeDM = RawAccessHandler.GetLastOutput();
		if (WholeDM) {
			std::cout << "DM ok" << std::endl;

			double FrameTimestamp = -1;
			std::string FrameStartTimePath = "User/FrameStartTime.double";
			if (WholeDM->isExistingLeaf(FrameStartTimePath) && !WholeDM->operator[](FrameStartTimePath).isNull()) {
				FrameTimestamp = WholeDM->GetLeaf(FrameStartTimePath).Ref<double>();

				if ((index > 0) && (lastTimestamp > FrameTimestamp)) return false;
				lastTimestamp = FrameTimestamp;
			}
			else {
				std::cout << "FrameTimestamp missing!!!" << std::endl;
				return false;
			}



			//cv::Mat IndexMap = WholeDM->GetLeaf(IndexMapPath).Ref<cv::Mat>();
			//std::cout << "IIIIIIIIIIIIIndexMap rows:" << IndexMap.rows << std::endl;

			//WholeDM->SaveToFolderStructure("dm");

			/*
			//cv::Mat transf34 = WholeDM->GetLeaf("Global/Settings/Coordinates/CoordinateTransformation.Mat").Ref<cv::Mat>();
			cv::Mat transf34 = WholeDM->GetLeaf("Global/Settings/Coordinates/CoordinateSpaces/MarkerSpace.Mat").Ref<cv::Mat>();
			cv::Mat t44(4, 4, CV_32F);
			t44 = 0;
			transf34.copyTo(t44(cv::Rect(0, 0, transf34.cols, transf34.rows)));
			cv::Matx44f transf44 = t44.clone();
			transf44(0, 3) *= 0.001f;
			transf44(1, 3) *= 0.001f;
			transf44(2, 3) *= 0.001f;
			transf44(3, 3) = 1.0f;
			*/

			/*
			cv::Mat transf2 = WholeDM->GetLeaf("Global/Settings/Coordinates/CoordinateSpaces/MarkerSpace.Mat").Ref<cv::Mat>();
			cv::Matx44f transf44_2 = transf;
			std::cout << "transf44_2: " << transf44_2 << std::endl;
			*/

			cv::Mat Texture = WholeDM->GetLeaf(TexturePath).Ref<cv::Mat>();
			sd.m_colorWidth = Texture.cols;
			sd.m_colorHeight = Texture.rows;
			std::cout << "Texture rows:" << Texture.rows << std::endl;
			cv::Mat Depth = WholeDM->GetLeaf(DepthPath).Ref<cv::Mat>();
			cv::Mat PointCloud = WholeDM->GetLeaf(PointCloudPath).Ref<cv::Mat>();
			photoneoTools::CameraCalibration32 cameraCalibration;
			cameraCalibration.SetCameraMatrix(1824.885927 / 3.0*2.0, 1824.728925 / 3.0*2.0, 878.237084 / 3.0*2.0, 594.768363 / 3.0*2.0);

			cv::Mat Texture32In;
			Texture.convertTo(Texture32In, CV_32F);
			cv::Mat Texture32Out;

			std::cout << Depth.size() << std::endl;

			if (!ReprojectToDepthMap<float>(cameraCalibration, PointCloud, Depth.size(), Depth, Texture32In, Texture32Out, 0.005)){
				std::cout << "nevyslo to" << std::endl;
			}
			std::cout << Depth.size() << std::endl;

			std::cout << "TextureOut rows: " << Texture32Out.rows << std::endl;

			Texture32Out.convertTo(Texture, CV_16U);

			sd.m_depthWidth = Depth.cols;
			sd.m_depthHeight = Depth.rows;
			Depth *= 10.0f;
			std::cout << "Depth rows:" << Depth.rows << std::endl;

			sd.m_intrinsic = cv::Matx44f::eye();
			sd.m_extrinsic = cv::Matx44f::eye();

			//cv::Matx33f camMat = 
			/*
			std::string CamMatPath = "Global/Cameras/0/Calibration/CameraMatrix";
			sd.m_intrinsic(0, 0) = WholeDM->GetLeaf(CamMatPath + "/0").Ref<double>();
			sd.m_intrinsic(0, 2) = WholeDM->GetLeaf(CamMatPath + "/2").Ref<double>();
			sd.m_intrinsic(1, 1) = WholeDM->GetLeaf(CamMatPath + "/4").Ref<double>();
			sd.m_intrinsic(1, 2) = WholeDM->GetLeaf(CamMatPath + "/5").Ref<double>();
			std::cout << "CamMat0:" << WholeDM->GetLeaf(CamMatPath + "/0").Ref<double>() << std::endl;
			*/
			/*
			sd.m_intrinsic(0, 0) = 2269.403143;
			sd.m_intrinsic(0, 2) = 1051.710324;
			sd.m_intrinsic(1, 1) = 2269.648174;
			sd.m_intrinsic(1, 2) = 738.025951;
			*/
			/*
			sd.m_intrinsic(0, 0) = 2244.757168;
			sd.m_intrinsic(0, 2) = 1014.823470;
			sd.m_intrinsic(1, 1) = 2244.139312;
			sd.m_intrinsic(1, 2) = 768.641646;
			*/
			sd.m_intrinsic(0, 0) = 1824.885927 / 3.0*2.0;
			sd.m_intrinsic(0, 2) = 878.237084 / 3.0*2.0;
			sd.m_intrinsic(1, 1) = 1824.728925 / 3.0*2.0;
			sd.m_intrinsic(1, 2) = 594.768363 / 3.0*2.0;

			RGBDFrame frame;
			/*
			if (index == 0) frame.m_cameraToWorld = transf44;
			else
			{
			//cv::Matx33f m33((float*)m.ptr());
			//cv::Mat m = .;
			cv::Mat inv = ComputeInverseCoordinateSpaceTransformation(cv::Mat(sd.m_frames[0].m_cameraToWorld));
			//(float*)m.ptr()
			cv::Mat inv2 = ComputeInverseCoordinateSpaceTransformation(cv::Mat(transf44));

			//cv::Mat res = cv::Mat(transf44)*(inv);
			cv::Mat res = inv*cv::Mat(transf44);
			//cv::Mat res = cv::Mat(sd.m_frames[0].m_cameraToWorld)*inv2;
			//cv::Mat res = inv2*(cv::Mat(sd.m_frames[0].m_cameraToWorld));

			frame.m_cameraToWorld = res;

			}
			*/
			//std::cout << "transf: " << frame.m_cameraToWorld << std::endl;

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
		}
		else {
			std::cout << "WholeDM = 0" << std::endl;
			return false;
		}

	}
	else {
		std::cout << "Failed to retrieve the frame!";
		return false;
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

void JanTest()
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
}


int main(int argc, char *argv[]) {

	JanTest();

	return 1;


}