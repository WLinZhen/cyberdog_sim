
#ifndef CYBER_GUIDE_IOINTERFACE_H
#define CYBER_GUIDE_IOINTERFACE_H

#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "message/UserParameters.h"
#include <string>

class IOInterface{
public:
    IOInterface(){}
    ~IOInterface(){
        //delete cmdPanel;
    }
    virtual void sendRecv(const LowlevelCmd *cmd, LowlevelState *state) = 0;
    bool _userFlag;

protected:
    //CmdPanel *;
};

#endif //CYBER_GUIDE_IOINTERFACE_H
