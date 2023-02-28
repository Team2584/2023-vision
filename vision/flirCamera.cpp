#include "Cameras.h"

using namespace std;
using namespace cv;

flirCamera::flirCamera(std::string camSerial)
    : flirSystem{}, flirCamList{},
      colorFrame{cv::Size(width, height), CV_8UC3}, grayFrame{cv::Size(width, height), CV_8UC1}
{
    using namespace Spinnaker;
    using namespace Spinnaker::GenApi;
    using namespace Spinnaker::GenICam;

    flirSystem = Spinnaker::System::GetInstance();
    flirCamList = flirSystem->GetCameras();

    const int numCameras = flirCamList.GetSize();

    // Set info
    setDistCoeffs(-0.4435755056, 0.335254035289252, 0.0002605483032105911, 0.001042419159982218,
                  0.2248450637613325);
    setCamParams(571, 572, 346, 247);

    // Finish if there are no cameras
    if (numCameras == 0)
    {
        flirCamList.Clear();
        flirSystem->ReleaseInstance();
        cout << "No flir cameras found" << endl;
        // TODO throw
    }

    // Select camera
    pCam = flirCamList.GetBySerial(camSerial);
    pCam->Init();

    // Retrieve GenICam nodemap
    INodeMap &nodeMap = pCam->GetNodeMap();

    // Recieve enumeration node from nodemap
    CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
    if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
    {
        cout << "Unable to set acquisition mode to continuous (enum retrieval). Aborting..." << endl
             << endl;
        // TODO throw
    }

    // Retrieve entry node from enumeration node
    CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
    if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
    {
        cout << "Unable to set acquisition mode to continuous (entry retrieval). Aborting..."
             << endl
             << endl;
        // TODO throw
    }

    // Retrieve integer value from entry node
    const int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();

    // Set integer value from entry node as new value of enumeration node
    ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

    // Retrieve Stream Parameters device nodemap
    Spinnaker::GenApi::INodeMap &sNodeMap = pCam->GetTLStreamNodeMap();

    // Retrieve Buffer Handling Mode Information
    CEnumerationPtr ptrHandlingMode = sNodeMap.GetNode("StreamBufferHandlingMode");
    if (!IsAvailable(ptrHandlingMode) || !IsWritable(ptrHandlingMode))
    {
        cout << "Unable to set Buffer Handling mode (node retrieval). Aborting..." << endl << endl;
        // TODO throw
    }

    CEnumEntryPtr ptrHandlingModeEntry = ptrHandlingMode->GetCurrentEntry();
    if (!IsAvailable(ptrHandlingModeEntry) || !IsReadable(ptrHandlingModeEntry))
    {
        cout << "Unable to set Buffer Handling mode (Entry retrieval). Aborting..." << endl << endl;
        // TODO throw
    }

    // Set buffer handling mode
    ptrHandlingModeEntry = ptrHandlingMode->GetEntryByName("NewestOnly");
    ptrHandlingMode->SetIntValue(ptrHandlingModeEntry->GetValue());

    // Exposure
    pCam->ExposureAuto.SetValue(ExposureAuto_Continuous);

    // Gain
    pCam->GainAuto.SetValue(GainAutoEnums::GainAuto_Once);

    // Gamma
    pCam->Gamma.SetValue(1.0);

    pCam->BeginAcquisition();
}

flirCamera::~flirCamera()
{
    pCam = nullptr;
    flirCamList.Clear();
    flirSystem->ReleaseInstance();
}

void flirCamera::setAutoExposure()
{
    pCam->ExposureAuto.SetValue(Spinnaker::ExposureAuto_Continuous);
}

void flirCamera::setManualExposure(int exposuretime = 0)
{
    // If no argument passed, use auto_once mode (this is the default)
    if (exposuretime == 0)
    {
        pCam->ExposureAuto.SetValue(Spinnaker::ExposureAuto_Once);
    }
    else
    {
        pCam->ExposureAuto.SetValue(Spinnaker::ExposureAuto_Off);
        pCam->ExposureMode.SetValue(Spinnaker::ExposureMode_Timed);
        pCam->ExposureTime.SetValue(exposuretime);
    }
}

void flirCamera::setManualGain(double value = 0)
{
    // If no argument passed, use auto_once mode (this is the default)
    if (value == 0)
    {
        pCam->GainAuto.SetValue(Spinnaker::GainAutoEnums::GainAuto_Once);
    }
    else
    {
        pCam->GainAuto.SetValue(Spinnaker::GainAutoEnums::GainAuto_Off);
        pCam->Gain.SetValue(value);
    }
}

void flirCamera::setAutoGain()
{
    pCam->GainAuto.SetValue(Spinnaker::GainAutoEnums::GainAuto_Continuous);
}

void flirCamera::getFrame()
{
    Spinnaker::ImagePtr frame = nullptr;
    Spinnaker::ImageStatus framestatus;
    frame = pCam->GetNextImage();

    framestatus = frame->GetImageStatus();
    if (framestatus != 0)
        printf("FRAME ERROR");

    grayFrame.data = (uint8_t *)frame->GetData();
    cv::cvtColor(grayFrame, colorFrame, cv::COLOR_GRAY2BGR);
    frame->Release();
}
