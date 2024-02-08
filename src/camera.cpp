//#include "PvSampleUtils.h"
#include "PvDevice.h"
#include "PvStream.h"
#include "PvPipeline.h"

/*
class CameraStream
{
public:
    CameraStream(const std::string& id) : 
        mDevice(PvDevice::CreateAndConnect(id)),
        mStream(PvStream::CreateAndOpen()),
        mPipeline(new PvPipeline(mDevice))
    {
        mDeviceParams = mDevice->GetParameters();
    }

    ~CameraStream()
    {
        delete mPipeline;
        mStream->Close();
        PvStream::Free(mStream);
        mDevice->Disconnect();
        PvDevice::Free(mDevice);
    }

    void Start()
    {
        mPipeline->Start();
        mDeviceParams->SetEnumValue("AcquisitionStart", "Start");
    }

    void Stop()
    {
        mDeviceParams->SetEnumValue("AcquisitionStop", "Stop");
        mPipeline->Stop();
    }

    PvBuffer* RetrieveBuffer()
    {
        PvBuffer* pBuffer = NULL;
        PvResult  lOperationResult;
        mPipeline->RetrieveNextBuffer(&pBuffer, 1000, &lOperationResult);
        return pBuffer;
    }

private:
    PvDevice* mDevice;
    PvStream* mStream;
    PvPipeline* mPipeline;
    PvGenParameterArray* mDeviceParams;
};
*/

int main() {
    return 0;
}