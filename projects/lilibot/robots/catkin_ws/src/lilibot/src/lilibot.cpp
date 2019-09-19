#include "lilibot.h"
namespace lilibot_ns{

    Lilibot::Lilibot(){
        dxl_wb_ = new DynamixelWorkbench;
        ser = new serial::Serial();
    }

    Lilibot::~Lilibot(){
        for (uint8_t index = 0; index < dxl_cnt_; index++)
            dxl_wb_->itemWrite(dxl_id_[index], "Torque_Enable", 0);
        delete dxl_wb_;
        delete ser;
    }

    bool Lilibot::init(ros::NodeHandle* node_handle){
        //1) init dynamixel
        node=node_handle;
        string dxl_device;
        if(!node->getParam("dxl_device",dxl_device)){
            ROS_ERROR("No dxl_device given (namespace: %s)", node->getNamespace().c_str());
            return false;
        }
        int dxl_baud;
        if(!node->getParam("dxl_baud",dxl_baud)){

            ROS_ERROR("No dxl_baud given (namespace: %s)", node->getNamespace().c_str());
            return false;
        }
        int scan_range;
        if(!node->getParam("scan_range",scan_range)){
            ROS_ERROR("No scan_range given (namespace: %s)", node->getNamespace().c_str());
            return false;
        }
        int profile_velocity;
        if(!node->getParam("profile_velocity",profile_velocity)){
            ROS_ERROR("No profile velocity given (namespace: %s)", node->getNamespace().c_str());
            return false;

        }
        int profile_acceleration;
        if(!node->getParam("profile_acceleration",profile_acceleration)){
            ROS_ERROR("No profile acclearation given (namespace: %s)", node->getNamespace().c_str());
            return false;
        }
        string imu_device;
        if(!node->getParam("imu_device",imu_device)){
            ROS_ERROR("No imu device given (namespace: %s)", node->getNamespace().c_str());
            return false;
        }
        int imu_baud;
        if(!node->getParam("imu_baud",imu_baud)){
            ROS_ERROR("No imu baud given (namespace: %s)", node->getNamespace().c_str());
            return false;
        }
        
        if(!node->getParam("leg_num",leg_num)){
            ROS_ERROR("No leg_num given (namespace: %s)", node->getNamespace().c_str());
            return false;
        }
        
        if(!node->getParam("motor_num",motor_num)){
            ROS_ERROR("No motor_num given (namespace: %s)", node->getNamespace().c_str());
            return false;
        }

        if(!node->getParam("sensor_num",sensor_num)){
            ROS_ERROR("No sensor_num given (namespace: %s)", node->getNamespace().c_str());
            return false;
        }
        
         if(!node->getParam("ori_num",ori_num)){
            ROS_ERROR("No ori_num given (namespace: %s)", node->getNamespace().c_str());
            return false;
        }
        
        // open motor
        dxl_wb_->begin(dxl_device.c_str(), dxl_baud);
        if (dxl_wb_->scan(dxl_id_, &dxl_cnt_, scan_range) != true) {
            ROS_ERROR("Not found Motors, Please check scan range or baud rate");
            ros::shutdown();
            return false;
        }
        initMsg();
        uint32_t driven_type =X_SERIES_CURRENT_BASED_POSITION_CONTROL_MODE;
        for (int index = 0; index < dxl_cnt_; index++){
            dxl_wb_->jointMode(dxl_id_[index], profile_velocity, profile_acceleration);
            dxl_wb_->itemWrite(dxl_id_[index], "Torque_Enable", 0);
            dxl_wb_->itemWrite(dxl_id_[index],"Operating_Mode", driven_type);
            dxl_wb_->itemWrite(dxl_id_[index], "Torque_Enable", 1);
        }

        dxl_wb_->addSyncWrite("Goal_Position");
        dxl_wb_->addSyncWrite("Goal_Current");
        dxl_wb_->addSyncRead("Present_Position");
        dxl_wb_->addSyncRead("Present_Velocity");
        dxl_wb_->addSyncRead("Present_Current");
        //2) init orientation sensor
        try 
        { 
            //设置串口属性，并打开串口 
            ser->setPort(imu_device.c_str()); 
            ser->setBaudrate(imu_baud); 
            serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
            ser->setTimeout(to); 
            ser->open(); 
        } 
        catch (serial::IOException& e) 
        { 
            ROS_ERROR_STREAM("Unable to open port :JY901"); 
            return false; 
        }    
        //检测串口是否已经打开，并给出提示信息 
        if(ser->isOpen()) 
        { 
            ROS_INFO_STREAM("Serial Port initial successful!"); 
        } 
        else 
        { 
            return true; 
        }
        goal_pos.resize(dxl_cnt_);
        goal_vel.resize(dxl_cnt_);
        goal_cur.resize(dxl_cnt_);

        pre_pos.resize(dxl_cnt_);
        pre_vel.resize(dxl_cnt_);
        pre_cur.resize(dxl_cnt_);
        //local control
        pos_err.resize(dxl_cnt_);
        pre_pos_err.resize(dxl_cnt_);
        calc_tor.resize(dxl_cnt_);
        goal_tor.resize(dxl_cnt_);
        // sensory feedback
        position.resize(motor_num);
        velocity.resize(motor_num);
        current.resize(motor_num);
        grf.resize(leg_num);
        ori.resize(ori_num);
        motorValue.resize(motor_num);
        sensorValue.resize(sensor_num);
        // wait
        ros::Duration(3.5).sleep();
        p_gain=0.001;
        d_gain=0.001;
        MI=0.02;
        t=0;
        return true;
    }

    void Lilibot::readJoints(){
        //1) read joint angles
        int32_t* temp_pos;
        temp_pos = dxl_wb_->syncRead("Present_Position");
        for(int index=0;index < dxl_cnt_;index++){
            pre_pos[index]=temp_pos[index];
            position.at(index) = dxl_wb_->convertValue2Radian(dxl_id_[index],pre_pos[index]);
        }
        //2) read joint velocity
        int32_t* temp_vel;
        temp_vel = dxl_wb_->syncRead("Present_Velocity");
        for(int index=0;index < dxl_cnt_;index++){
            pre_vel[index]=temp_vel[index];
            velocity.at(index) = dxl_wb_->convertValue2Velocity(dxl_id_[index],pre_vel[index]);
        }
        //3) read force of foot
        int32_t* temp_cur;
        temp_cur = dxl_wb_->syncRead("Present_Current");
        for(int index=0;index<dxl_cnt_;index++){
            pre_cur[index]=temp_cur[index];
            current[index] = dxl_wb_->convertValue2Torque(dxl_id_[index],pre_cur[index]);
        }
    }

    void Lilibot::readImu(){
        std_msgs::String results;
        uint8_t sum=0x00;
        uint8_t temp1[11];
        signed short temp2[6];
        float roll=0.0,pitch=0.0,yaw=0.0;
        results.data=ser->read(ser->available());
        if((results.data[0]==0x55)&&(results.data[1]==0x53)){
            for(uint8_t i=0;i<11;i++)
                temp1[i] = (uint8_t)results.data[i];                   
            for(uint8_t i=0;i<10;i++){
                sum+=temp1[i];
            }

            for(uint8_t i=0;i<6;i++)
                temp2[i]=(signed short)temp1[i+2];
            if(sum==temp1[10]){
                roll = (short)((temp2[1]<<8)|temp2[0])/32768.0*M_PI -  0.04;
                pitch = (short)((temp2[3]<<8)|temp2[2])/32768.0*M_PI + 0.055;
                yaw = (short)((temp2[5]<<8)|temp2[4])/32768.0*M_PI;
                ori.at(0)= -0.65*roll;
                ori.at(1) = -1.0*pitch;
                ori.at(2) = -1.0*yaw;
            }
        }
    }

    void Lilibot::readFootForce(){
        
        // for F and B structure configuration //
        float grf_temp;
        float grf_offset[] ={-0.3, -0.3, -0.3,-0.3};
        float grf_slope = 1.1;
        float grf_uplimit = 1.2;//0.75
        float temp_off=(MI-0.045)*10.0;
        grf_offset[0]=grf_offset[0]- (temp_off>0.0 ? temp_off: 0.0);
        grf_offset[1]=grf_offset[1]- (temp_off>0.0 ? temp_off: 0.0);
        grf_offset[2]=grf_offset[2]- (temp_off>0.0 ? temp_off: 0.0);
        grf_offset[3]=grf_offset[3]- (temp_off>0.0 ? temp_off: 0.0);

        grf_temp = grf_slope*current[2]+grf_offset[0];    
        grf[0] = grf_temp < 0.0 ? 0.0:(grf_temp > grf_uplimit ? grf_uplimit:grf_temp);//right

        grf_temp = grf_slope*current[5] + grf_offset[1];
        grf[1] = grf_temp < 0.0 ? 0.0:(grf_temp > grf_uplimit ? grf_uplimit:grf_temp);//right

        grf_temp = grf_slope*-1.0*current[8] + grf_offset[2];
        grf[2] = grf_temp < 0.0 ? 0.0:(grf_temp > grf_uplimit ? grf_uplimit:grf_temp);// left 

        grf_temp = grf_slope*-1.0*current[11]+ grf_offset[3];
        grf[3] = grf_temp < 0.0 ? 0.0:(grf_temp > grf_uplimit ? grf_uplimit:grf_temp);//left
    /*
    
        //  for O structure configuration  //
        float grf_temp;
        float grf_offset[] ={-0.3, -0.3, -0.3,-0.3};
        float grf_slope[] = {1.50,1.05,1.50,1.05};
        float grf_uplimit = 1.2;//0.75
        float temp_off=(MI-0.045)*10.0;
        grf_offset[0]=grf_offset[0]- (temp_off>0.0 ? temp_off: 0.0);
        grf_offset[1]=grf_offset[1]- (temp_off>0.0 ? temp_off: 0.0);
        grf_offset[2]=grf_offset[2]- (temp_off>0.0 ? temp_off: 0.0);
        grf_offset[3]=grf_offset[3]- (temp_off>0.0 ? temp_off: 0.0);

        grf_temp = grf_slope[0]*-1.0*current[1]+grf_offset[0];    
        grf[0] = grf_temp < 0.0 ? 0.0:(grf_temp > grf_uplimit ? grf_uplimit:grf_temp);//right

        grf_temp = grf_slope[1]*1.0*current[5] + grf_offset[1];
        grf[1] = grf_temp < 0.0 ? 0.0:(grf_temp > grf_uplimit ? grf_uplimit:grf_temp);//right

        grf_temp = grf_slope[2]*current[7] + grf_offset[2];
        grf[2] = grf_temp < 0.0 ? 0.0:(grf_temp > grf_uplimit ? grf_uplimit:grf_temp);// left 

        grf_temp = grf_slope[3]*-1.0*current[11]+ grf_offset[3];
        grf[3] = grf_temp < 0.0 ? 0.0:(grf_temp > grf_uplimit ? grf_uplimit:grf_temp);//left
    
        // for X structure configuration  //
        float grf_temp;
        float grf_offset[] ={-0.3, -0.3, -0.3,-0.3};
        float grf_slope[] = {1.1,1.1,1.1,1.1};
        float grf_uplimit = 1.2;//0.75
        float temp_off=(MI-0.045)*10.0;
        grf_offset[0]=grf_offset[0]- (temp_off>0.0 ? temp_off: 0.0);
        grf_offset[1]=grf_offset[1]- (temp_off>0.0 ? temp_off: 0.0);
        grf_offset[2]=grf_offset[2]- (temp_off>0.0 ? temp_off: 0.0);
        grf_offset[3]=grf_offset[3]- (temp_off>0.0 ? temp_off: 0.0);

        grf_temp = grf_slope[0]*current[2]+grf_offset[0];    
        grf[0] = grf_temp < 0.0 ? 0.0:(grf_temp > grf_uplimit ? grf_uplimit:grf_temp);//right

        grf_temp = grf_slope[1]*-1.0*current[5] + grf_offset[1];
        grf[1] = grf_temp < 0.0 ? 0.0:(grf_temp > grf_uplimit ? grf_uplimit:grf_temp);//right

        grf_temp = grf_slope[2]*current[8] + grf_offset[2];
        grf[2] = grf_temp < 0.0 ? 0.0:(grf_temp > grf_uplimit ? grf_uplimit:grf_temp);// left 

        grf_temp = grf_slope[3]*current[10]+ grf_offset[3];
        grf[3] = grf_temp < 0.0 ? 0.0:(grf_temp > grf_uplimit ? grf_uplimit:grf_temp);//left
    */
    }


    void Lilibot::initMsg(){
        printf("-----------------------------------------------------------------------\n");
        printf("  dynamixel_workbench controller; position control example             \n");
        printf("-----------------------------------------------------------------------\n");
        printf("\n");

        for (int index = 0; index < dxl_cnt_; index++)
        {
            printf("MODEL   : %s\n", dxl_wb_->getModelName(dxl_id_[index]));
            printf("ID      : %d\n", dxl_id_[index]);
            printf("dxl_cnt_t:%d\n",dxl_cnt_);
            printf("\n");
        }
        printf("-----------------------------------------------------------------------\n");
    }


    //public----------------------------------------
    void Lilibot::getSensorValue(vector<sensor>& value){
        assert(value.size()==sensor_num);
        readFootForce();
        readImu();
        readJoints();
        for(uint8_t idx=0;idx<motor_num;idx++){
            value.at(idx)=position.at(idx);
        }
        for(uint8_t idx=0;idx<leg_num;idx++){
            value.at(idx+motor_num)=grf.at(idx);
        }
        for(uint8_t idx=0;idx<ori_num;idx++){
            value.at(idx+motor_num+leg_num)=ori.at(idx);
        }
        for(uint8_t idx=0;idx<leg_num;idx++){
            value.at(idx+motor_num+leg_num+ori_num)=0.0;
        }

    }


    void Lilibot::setMotorValue(const vector<command>& cmd_pos,const  vector<command>& cmd_vel,const vector<command>& cmd_eff){
        assert(cmd_pos.size()>=dxl_cnt_);
        assert(cmd_vel.size()>=dxl_cnt_);
        for(int index=0;index<dxl_cnt_;index++){
            goal_pos[index]=dxl_wb_->convertRadian2Value(dxl_id_[index],cmd_pos[index]);
            goal_vel[index]=dxl_wb_->convertVelocity2Value(dxl_id_[index],cmd_vel[index]);
            goal_cur[index]=dxl_wb_->convertTorque2Value(dxl_id_[index],cmd_eff[index]);
        }
    }

    void Lilibot::setMotorValue(const vector<command>& value){
        assert(value.size()>=dxl_cnt_);
        for(int index=0;index<dxl_cnt_;index++){
            //goal_pos[index]=dxl_wb_->convertRadian2Value(dxl_id_[index],value[index]);
            value2cmd(value,goal_pos);
        }
        t++;
        if(t>1000000)
            t=0;
        if(t%100==0){
            getParameters();
        }

     //   cout<<"Time:"<<ros::Time::now()<<endl;
        localController();

    }
    void Lilibot::getParameters(){
        if(!node->getParam("p_gain",p_gain))
            ROS_ERROR("can't loccate p_gain");
        if(!node->getParam("d_gain",d_gain))
            ROS_ERROR("can't loccate d_gain");
        if(!node->getParam("MI",MI))
            ROS_ERROR("can't loccate MI");
    }

    void Lilibot::localController(){

        for(uint8_t index =0; index < dxl_cnt_;index++){
            pos_err[index] = goal_pos[index]-pre_pos[index];
            calc_tor[index]  = p_gain*pos_err[index] + d_gain*(pos_err[index]-pre_pos_err[index]);
            goal_tor[index]  = (int32_t)(dxl_wb_->convertTorque2Value(dxl_id_[index] , calc_tor[index]));
            pre_pos_err[index] = pos_err[index];
        }

              writeMotorValue(goal_pos,goal_tor);

    }

    void Lilibot::writeMotorValue(vector<int32_t>& pos,vector<int32_t>& eff){
        assert(pos.size()>=dxl_cnt_);
        assert(eff.size()>=dxl_cnt_);
        if(dxl_wb_->syncWrite("Goal_Current", &eff[0])==false)
            ROS_ERROR("fail to drive motor");

        if(dxl_wb_->syncWrite("Goal_Position",&pos[0])==false)
            ROS_ERROR("fail to drive motor");
    }

    void Lilibot::value2cmd(const std::vector<float>& value, std::vector<int32_t>& cmd){
        assert(value.size()==motor_num);
        assert(cmd.size()>=dxl_cnt_);
        for(int index=0;index<dxl_cnt_;index++){
            switch(index){
                case 0:
                case 3:
                    cmd[index]=(unsigned int )floor(-350.0*value.at(index)+2048.0);
                    break;
                case 1:
                case 4:
                    cmd[index]=(unsigned int )floor(512.0*(value.at(index)+0.25)+512.0);//+0.3
                    break;
                case 2:
                case 5:
                    cmd[index]=(unsigned int )floor(-724.0*(value.at(index)+0.25)+1324.0);//+0.2
                    break;
                case 6:
                case 9:
                    cmd[index]=(unsigned int )floor(350.0*value.at(index)+2048.0);
                    break;
                case 7:
                case 10: 
                    cmd[index]=(unsigned int )floor(-512.0*(value.at(index)+0.25)+3583.0);//0.3
                    break;
                case 8:
                case 11:
                    cmd[index]=(unsigned int )floor(724.0*(value.at(index)+0.25)+2772.0);//+0.2
                    break;
                default:
                    perror("error value2int");
            }

        }
    }
}
