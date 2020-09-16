#include "ststick.h"
using namespace std;
StStick::StStick(){
    joystick = new Joystick;
    if (!joystick->isFound()){
        perror("Joystick open failed.\n");
    }else{
        printf("Joystick open successful.\n");
    }
}

StStick::~StStick(){

    delete joystick;
}
bool StStick::init(ros::NodeHandle* node){

    node_handle= node;
    GCommand.event=false;
    GCommand.isButton=false;
    GCommand.isAxis=false;    
    return true;
}


void StStick::stickEvent(){
    //1) read the input command from human
    GCommand.event=false;
    GCommand.isButton=false;
    GCommand.isAxis=false;
    GCommand.button=-1;
    GCommand.axis=-1;
    GCommand.valueOffset=0.0;
    JoystickEvent event;
    if (joystick->sample(&event)) {
        if (event.isButton()) {
            if(event.value == 1) {
                GCommand.event=true;
                GCommand.isButton=true;
                GCommand.isAxis=false;
                switch(event.number){
                    case 0:
                        GCommand.button=0;
                        break;  
                    case 1:
                        GCommand.button=1;
                        break;  
                    case 2:
                        GCommand.button=2;
                        break;
                    case 3:
                        GCommand.button=3;
                        break;
                    case 4:
                        GCommand.button=4;
                        break;
                    case 5:
                        GCommand.button=5;
                        break;
                    case 6:
                        GCommand.button=6;
                        break;
                    case 7:
                        GCommand.button=7;
                        break;
                    case 8:
                        GCommand.button=8;
                        break;
                    case 9:
                        GCommand.button=9;
                        break;
                    default:
                        ;
                }
            }
        }else if (event.isAxis()) {
            int signEventValue = copysign(1.0, event.value);
            double eventValueOffset = 0;
            if(abs(event.value) >= 2580)
                eventValueOffset = signEventValue*((abs(event.value) - 2580) / (32767.0-2580));
            GCommand.event=true;
            GCommand.isAxis=true;
            GCommand.isButton=false;
            if((eventValueOffset==1)||(eventValueOffset==-1)){
                GCommand.valueOffset=eventValueOffset;
                switch(event.number){
                    case 0:
                        GCommand.axis=0;
                        break;
                    case 1:
                        GCommand.axis=1;
                        break;
                    case 2:
                        GCommand.axis=2;
                        break;
                    case 3:
                        GCommand.axis=3;
                        break;
                    case 4:
                        GCommand.axis=4;
                        break;
                    case 5:
                        GCommand.axis=5;
                        break;
                    case 6:
                        GCommand.axis=6;
                        break;
                    case 7:
                        GCommand.axis=7;
                        break;
                    default:
                        ;
                }
            }

        }
    }
}

void StStick::guide(){

    stickEvent();
    float temp;
    int cpg_type;
    if(GCommand.event){
        if(GCommand.isButton){
            switch(GCommand.button){
                case 0:
                    node_handle->getParam("MI",temp);
                    if((temp>0.00)&&(temp<0.25)){
                        node_handle->setParam("MI",temp-0.01);
                        cout<<"-> decrese mi:"<<temp-0.01<<endl;
                    }
                    break;  
                case 1:
                    node_handle->getParam("VRN_hip_L1",temp);
                    if((temp<0.09)&&(temp>0.01)){
                        node_handle->setParam("VRN_hip_L1",temp - 0.01);
                    }

                    node_handle->getParam("VRN_hip_L2",temp);
                    if((temp<0.09)&&(temp>0.01)){
                        node_handle->setParam("VRN_hip_L2",temp - 0.01);
                    }
                    
                    node_handle->getParam("VRN_hip_L3",temp);
                    if((temp<0.09)&&(temp>0.01)){
                        node_handle->setParam("VRN_hip_L3",temp - 0.01);
                    }
                    node_handle->getParam("VRN_hip_L4",temp);
                    if((temp<0.09)&&(temp>0.01)){
                        node_handle->setParam("VRN_hip_L4",temp - 0.01);
                        cout<<"-> decrease hip joint"<<temp-0.01<<endl;
                    }
                    break;  
                case 2:
                    node_handle->getParam("VRN_hip_L1",temp);
                    if((temp<0.08)&&(temp>0.0))
                        node_handle->setParam("VRN_hip_L1",temp + 0.01);
                    node_handle->getParam("VRN_hip_L2",temp);
                    if((temp<0.08)&&(temp>0.0))
                        node_handle->setParam("VRN_hip_L2",temp + 0.01);
                    node_handle->getParam("VRN_hip_L3",temp);
                    if((temp<0.08)&&(temp>0.0))
                        node_handle->setParam("VRN_hip_L3",temp + 0.01);
                    node_handle->getParam("VRN_hip_L4",temp);
                    if((temp<0.08)&&(temp>0.0)){
                        node_handle->setParam("VRN_hip_L4",temp + 0.01);
                        cout<<"-> increase hip joint"<<temp+0.01<<endl;
                    }
                    break;
                case 3:
                    node_handle->getParam("MI",temp);
                    if((temp>-0.02)&&(temp<0.22)){
                        node_handle->setParam("MI",temp + 0.01);
                        cout<<"-> increase mi:"<<temp+0.01<<endl;
                    }
                    break;
                case 4:
                    node_handle->getParam("VRN_knee_L1",temp);
                    if((temp<0.08)&&(temp>0.0))
                        node_handle->setParam("VRN_knee_L1",temp + 0.01);
                    node_handle->getParam("VRN_knee_L2",temp);
                    if((temp<0.08)&&(temp>0.0))
                        node_handle->setParam("VRN_knee_L2",temp + 0.01);
                    node_handle->getParam("VRN_knee_L3",temp);
                    if((temp<0.08)&&(temp>0.0))
                        node_handle->setParam("VRN_knee_L3",temp + 0.01);

                    node_handle->getParam("VRN_knee_L4",temp);
                    if((temp<0.08)&&(temp>0.0)){
                        node_handle->setParam("VRN_knee_L4",temp + 0.01);
                        cout<<"-> increase knee joint"<<temp+0.01<<endl;
                    }
                    break;
                case 5:
                    node_handle->getParam("VRN_knee_L1",temp);
                    if((temp<0.1)&&(temp>0.01))
                        node_handle->setParam("VRN_knee_L1",temp - 0.01);
                    node_handle->getParam("VRN_knee_L2",temp);
                    if((temp<0.1)&&(temp>0.01))
                        node_handle->setParam("VRN_knee_L2",temp - 0.01);
                    node_handle->getParam("VRN_knee_L3",temp);
                    if((temp<0.1)&&(temp>0.01))
                        node_handle->setParam("VRN_knee_L3",temp - 0.01);

                    node_handle->getParam("VRN_knee_L4",temp);
                    if((temp<0.1)&&(temp>0.01)){
                        node_handle->setParam("VRN_knee_L4",temp - 0.01);
                        cout<<"-> decrease knee joint"<<temp-0.01<<endl;
                    }
                    break;
                case 6:
                    node_handle->getParam("CPGType",cpg_type);
                    if(cpg_type==0){
                        cpg_type=1;
                    }else if(cpg_type==1){
                        cpg_type=2;
                    }else if(cpg_type==2){
                        cpg_type=3;
                    }else if(cpg_type==3){
                        cpg_type=4;
                    }else if(cpg_type==4){
                        cpg_type=5;
                    }else if(cpg_type==5){
                        cpg_type=6;
                    }else if(cpg_type==6){
                        cpg_type=7;
                    }else{
                        cpg_type=0;
                    }
                    node_handle->setParam("CPGType",cpg_type);
                    cout<<"-> up set CPGType:"<<cpg_type<<endl;
                    break;
                case 7:
                    node_handle->getParam("CPGType",cpg_type);
                    if(cpg_type==0){
                        cpg_type=7;
                    }else if(cpg_type==1){
                        cpg_type=0;
                    }else if(cpg_type==2){
                        cpg_type=1;
                    }else if(cpg_type==3){
                        cpg_type=2;
                    }else if(cpg_type==4){
                        cpg_type=3;
                    }else if(cpg_type==5){
                        cpg_type=4;
                    }else if(cpg_type==6){
                        cpg_type=5;
                    }else{
                        cpg_type=6;
                    }
                    node_handle->setParam("CPGType",cpg_type);
                    cout<<"-> dwon set CPGType:"<<cpg_type<<endl;
                    break;
                case 8:
                    if(!node_handle->getParam("WalkingMode",temp))
                        ROS_INFO("can't get walking_mode");
                    node_handle->setParam("WalkingMode",0.0);
                    cout<<"-> now is walking"<<endl;
                    break;
                case 9:
                    node_handle->setParam("WalkingMode",1.0);
                        cout<<"-> stop -------- stop "<<endl;
                        ros::shutdown();
                    break;
                default:
                    ;
            }
        }else if (GCommand.isAxis){
            switch(GCommand.axis){
                case 6:
                    if(GCommand.valueOffset==1){
                        node_handle->getParam("VRN_hip_L1",temp);
                        if(temp>=0.02){//valueOffset
                            node_handle->setParam("VRN_hip_L1",temp - GCommand.valueOffset*0.02);
                            node_handle->setParam("VRN_hip_L2",temp - GCommand.valueOffset*0.02);
                        }
                        node_handle->getParam("VRN_hip_L3",temp);
                        if(temp<=0.08){
                            node_handle->setParam("VRN_hip_L3",temp + GCommand.valueOffset*0.02);
                            node_handle->setParam("VRN_hip_L4",temp + GCommand.valueOffset*0.02);
                            printf("-> turn right \n");
                        }
                    }
                    if(GCommand.valueOffset==-1){
                        node_handle->getParam("VRN_hip_L1",temp);
                        if(temp<=0.08){//valueOffset
                            node_handle->setParam("VRN_hip_L1",temp - GCommand.valueOffset*0.02);
                            node_handle->setParam("VRN_hip_L2",temp - GCommand.valueOffset*0.02);
                        }
                        node_handle->getParam("VRN_hip_L3",temp);
                        if(temp>=0.02){
                            node_handle->setParam("VRN_hip_L3",temp + GCommand.valueOffset*0.02);
                            node_handle->setParam("VRN_hip_L4",temp + GCommand.valueOffset*0.02);
                            printf("-> turn left\n");
                        }
                    }
                    break;
                case 7:
                    if(GCommand.valueOffset==-1){

                        node_handle->setParam("PSN_L1",0.0);
                        node_handle->setParam("PSN_L2",0.0);
                        node_handle->setParam("PSN_L3",0.0);
                        node_handle->setParam("PSN_L4",0.0);
                        node_handle->setParam("VRN_hip_L1",0.04);
                        node_handle->setParam("VRN_hip_L2",0.04);
                        node_handle->setParam("VRN_hip_L3",0.04);
                        node_handle->setParam("VRN_hip_L4",0.04);
                        node_handle->setParam("VRN_knee_L1",0.04);
                        node_handle->setParam("VRN_knee_L2",0.04);
                        node_handle->setParam("VRN_knee_L3",0.04);
                        node_handle->setParam("VRN_knee_L4",0.04);
                        printf("-> go straight\n");
                    }
                    if(GCommand.valueOffset==1){
                        node_handle->setParam("PSN_L1",1.0);
                        node_handle->setParam("PSN_L2",1.0);
                        node_handle->setParam("PSN_L3",1.0);
                        node_handle->setParam("PSN_L4",1.0);
                        node_handle->setParam("VRN_hip_L1",-0.04);
                        node_handle->setParam("VRN_hip_L2",-0.04);
                        node_handle->setParam("VRN_hip_L3",-0.04);
                        node_handle->setParam("VRN_hip_L4",-0.04);
                        node_handle->setParam("VRN_knee_L1",-0.04);
                        node_handle->setParam("VRN_knee_L2",-0.04);
                        node_handle->setParam("VRN_knee_L3",-0.04);
                        node_handle->setParam("VRN_knee_L4",-0.04);
                        printf("-> go back\n");
                    }
                    break;
                case 0:
                    printf("axis 0\n");
                    break;
                case 1:
                    printf("axis 1 \n");
                    break;

                case 3:
                    if(GCommand.valueOffset==1){
                        if(!node_handle->getParam("MNB2_L1",temp))
                            ROS_INFO("can't get MNB2");
                        if((temp>-0.4)&&(temp<0.4)){
                            node_handle->setParam("MNB2_L1",temp-0.02);
                            node_handle->setParam("MNB2_L2",temp-0.02);
                            node_handle->setParam("MNB2_L3",temp-0.02);
                            node_handle->setParam("MNB2_L4",temp-0.02);
                            cout<<"-> decrease MNB2:"<<temp-0.02<<endl;
                        }
                        if(!node_handle->getParam("MNB3_L1",temp))
                            ROS_INFO("can't get MNB3");
                        if((temp>-0.3)&&(temp<0.3)){
                            node_handle->setParam("MNB3_L1",temp-0.02);
                            node_handle->setParam("MNB3_L2",temp-0.02);
                            node_handle->setParam("MNB3_L3",temp-0.02);
                            node_handle->setParam("MNB3_L4",temp-0.02);
                            cout<<"-> decrease MNB3:"<<temp-0.02<<endl;
                        }
                    }
                    if(GCommand.valueOffset==-1){
                        node_handle->getParam("MNB2_L1",temp);
                        if((temp>-0.42)&&(temp<0.35)){
                            node_handle->setParam("MNB2_L1",temp+0.02);
                            node_handle->setParam("MNB2_L2",temp+0.02);
                            node_handle->setParam("MNB2_L3",temp+0.02);
                            node_handle->setParam("MNB2_L4",temp+0.02);
                            cout<<"-> increase M<NB2:"<<temp+0.02<<endl;
                        }
                        node_handle->getParam("MNB3_L1",temp);
                        if((temp>-0.32)&&(temp<0.25)){
                            node_handle->setParam("MNB3_L1",temp+0.02);
                            node_handle->setParam("MNB3_L2",temp+0.02);
                            node_handle->setParam("MNB3_L3",temp+0.02);
                            node_handle->setParam("MNB3_L4",temp+0.02);
                            cout<<"-> increase MNB3:"<<temp+0.02<<endl;
                        }
                    }
                    break;
                case 4:
                    if(GCommand.valueOffset==1){
                        if(!node_handle->getParam("p_gain",temp))
                            ROS_INFO("can't get p_gain");
                        if((temp>0.002)&&(temp<0.03)){
                            node_handle->setParam("p_gain",temp-0.001);
                            cout<<"-> decrease p_gain:"<<temp-0.001<<endl;
                        }
                        if(!node_handle->getParam("d_gain",temp))
                            ROS_INFO("can't get d_gain");
                        if((temp>0.002)&&(temp<0.03)){
                            node_handle->setParam("d_gain",temp-0.001);
                            cout<<"-> decrease d_gain:"<<temp-0.001<<endl;
                        }
                    }
                    if(GCommand.valueOffset==-1){
                        node_handle->getParam("p_gain",temp);
                        if((temp>0.00)&&(temp<0.02)){
                            node_handle->setParam("p_gain",temp+0.001);
                            cout<<"-> increase p_gain:"<<temp+0.001<<endl;
                        }
                        node_handle->getParam("d_gain",temp);
                        if((temp>0.00)&&(temp<0.02)){
                            node_handle->setParam("d_gain",temp+0.001);
                            cout<<"-> increase d_gain:"<<temp+0.001<<endl;
                        }
                    }
                    break;
                default:
                    ;
            }
        }
    }
}



