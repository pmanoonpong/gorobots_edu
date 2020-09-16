--*******************************************
--*                                         *
--*        Adaptive Locomotion Control      *
--*                                         *
--*******************************************
-- update: 03/09/2020
-- version: 1.0.0


--*******************************************
--*                                         *
--*              description                *
--*                                         *
--*******************************************
-- creating vrep simulation of the gecko robot and starting rosnode named "vrep_ros_interface"

--*******************************************
--*                                         *
--*             define variable             *
--*                                         *
--*******************************************
geckoTape_dist = 0.0025 -- maximum distance for enable adhesive force
epsilon = 0.001 -- very small value

--*******************************************
--*                                         *
--*          ros callback function          *
--*                                         *
--*******************************************
function body_cb(msg)
    data = msg.data
    for i=1,3,1 do
        bodySignal[i] = data[i]
    end
end

function cpg_cb(msg)
    data = msg.data
    cpgSignal[1] = data[1]
    cpgSignal[2] = data[2]
    cpgSignal[3] = data[3]
    cpgSignal[4] = data[4]
end

function real_force_cb(msg)
    data = msg.data
    for i=1,4,1 do
        real_forceData[i] = data[i]
    end
    realRobot = true
end

function real_bodyAng_cb(msg)
    data = msg.data
    for i=1,4,1 do
        real_bodyAng[i] = data[i]
    end
    realRobot = true
end

function incinationConc_cb(msg)
    data = msg.data
    incConc = data[1]
end

function motor_cb(msg)
    data = msg.data
    for i=1,16,1 do
        motorSignal[i] = data[i]
    end
end

function sim_motor_cb(msg)
    data = msg.data
    for i=1,16,1 do
        sim_motorSignal[i] = data[i]
    end
end

function stretching_conc_cb(msg)
    data = msg.data
    for i=1,4,1 do
        stretchingConc[i] = data[i]
    end
end

function stretching_cb(msg)
    data = msg.data
    for i=1,4,1 do
        stretchingSig[i] = data[i]
    end
end

function confidence_cb(msg)
    data = msg.data
    confidenceConc = data[1]
end

function efference_cb(msg)
    data = msg.data
    for i=1,16,1 do
        effeSignal[i] = data[i]
    end
end

function indicator_cb(msg)
    data = msg.data
    stab = data[1]
    harm = data[2]
end

function real_imu_cb(msg)
    data = msg.data
    accel[1] = data[1]
    accel[2] = data[2]
    accel[3] = data[3]
    gyroData[1] = data[4]
    gyroData[2] = data[5]
    gyroData[3] = data[6]
    realRobot = true
end

-- initialize section
function sysCall_init()

    --*******************************************
    --*                                         *
    --*         create global variable          *
    --*                                         *
    --*******************************************

    -- ***************************  neural control *********************************
    bodySignal = {0,0,0, 0,0,0} -- Body signal  [cpg , sine]
    cpgSignal = {0,0,0,0} -- CPG signal
    motorSignal = {0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0}
    sim_motorSignal = {0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0}
    effeSignal = {0,0,0,0}

    -- *********  convert motor signal to simulated motor signal *******************



    -- **********************  simulated object handle *****************************
    legName = {'lf','rf','rh','lh'}
    jointHandle = {{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}}
    footHandle = {0,0,0,0}
    forceHandle = {0,0,0,0}
    -- Create all object handles including joint, foot and force sensor

    -- *****************************  sensory signal *******************************
    forceData = {0,0,0,0} -- simulated force signal from foot contact sensor
    jointTorque = {0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0}
    real_forceData = {0,0,0,0}
    distance = {0,0,0,0} -- distance between foot and the surface
    accel = {0,0,0} -- acceleration from imu
    gyroData={0,0,0} -- angular vilocity from imu
    bodyAng = {0,0,0} -- body inclination of the robot
    real_bodyAng = {0,0,0}

    -- *************************  parameter for rostopic ****************************
    forceTopic = {} -- simulated force parameter
    imuTopic = {} -- simulated imu parameter including acceleration and angular vilocity
    simTimeTopic = {} -- simulated time
    simBodyAngTopic = {} -- simulated robot's body angle
    simInfraredTopic = {}  -- simulated infrared sensors

    -- ******************** artificial hormone concentration  ***********************
    stretchingConc = {0,0,0,0} -- stretching concentration
    confidenceConc = 0 -- confidence inspired concentration
    incConc = 0 -- body inclination concentration

    stretchingSig = {0,0,0,0} -- leg stretching signal


    -- ************************ performance measurement  ***************************
    stab = 0 -- stability
    harm = 0 -- harmony
    walking_dist = 0 -- distance

    --*******************************************
    --*                                         *
    --*          create object handle           *
    --*                                         *
    --*******************************************
    geckoHandle=sim.getObjectAssociatedWithScript(sim.handle_self)
    for i=1,4,1 do
        for j=1,4,1 do
            jointHandle[i][j] = simGetObjectHandle('joint'..tostring(j).."_"..legName[i])
        end
        footHandle[i] = simGetObjectHandle('pad_'..legName[i])
        forceHandle[i] = simGetObjectHandle('fc_'..legName[i])
    end
    body = simGetObjectHandle('body')
    floor = simGetObjectHandle('floor')
    jointB1 = simGetObjectHandle('joint_b1')
    jointB2 = simGetObjectHandle('joint_b2')
    jointB3 = simGetObjectHandle('joint_b3')

    -- get graph handle
    graph = simGetObjectHandle('graph')
    g_test = simGetObjectHandle('test')

    -- initial position
    body_init_posi = simGetObjectPosition(body,floor)

    -- gyro sensor
    modelBaseG=sim.getObjectAssociatedWithScript(sim.handle_self)
    refG=sim.getObjectHandle('GyroSensor_reference')
    uiG=simGetUIHandle('GyroSensor_UI')
    simSetUIButtonLabel(uiG,0,sim.getObjectName(modelBaseG))
    gyroCommunicationTube=sim.tubeOpen(0,'gyroData'..sim.getNameSuffix(nil),1)
    oldTransformationMatrix=sim.getObjectMatrix(refG,-1)
    lastTime=sim.getSimulationTime()

    -- accelero sensor
    modelBaseA=sim.getObjectAssociatedWithScript(sim.handle_self)
    massObject=sim.getObjectHandle('Accelerometer_mass')
    accSensor=sim.getObjectHandle('Accelerometer_forceSensor')
    _,mass=sim.getObjectFloatParameter(massObject,sim.shapefloatparam_mass)
    uiA=simGetUIHandle('Accelerometer_UI')
    simSetUIButtonLabel(uiA,0,sim.getObjectName(modelBaseA))
    accelCommunicationTube=sim.tubeOpen(0,'accelerometerData'..sim.getNameSuffix(nil),1)

    -- proximity  sensor
    irlSensor=sim.getObjectHandle("IRL") -- Handle of the left proximity sensor
    irrSensor=sim.getObjectHandle("IRR") -- Handle of the right proximity sensor

    --*******************************************
    --*                                         *
    --*              setup ros node             *
    --*                                         *
    --*******************************************

    -- Check if the required ROS plugin is loaded
    moduleName=0
    moduleVersion=0
    index=0
    pluginNotFound=true
    while moduleName do
        moduleName,moduleVersion=sim.getModuleName(index)
        if (moduleName=='ROSInterface') then
            pluginNotFound=false
        end
        index=index+1
    end
    if (pluginNotFound) then
        sim.displayDialog('Error','The RosInterface was not found.',sim.dlgstyle_ok,false,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
    end

    -- If found then start the subscribers and publishers
    if (not pluginNotFound) then


        -- ************************ publisher  ***************************

        forcePublisher = simROS.advertise('/sim_force_topic','std_msgs/Float32MultiArray')-- simulated force topic
        imuPublisher = simROS.advertise('/sim_imu_topic','std_msgs/Float32MultiArray') -- simulated imu topic
        simTimePublisher = simROS.advertise('/sim_time_topic','std_msgs/Float32MultiArray') -- simulated time topic
        simBodyAngPublisher = simROS.advertise('/sim_bodyAng_topic','std_msgs/Float32MultiArray') -- body inclination topic
        simInfraredPublisher = simROS.advertise('/sim_infrared_topic','std_msgs/Float32MultiArray') -- simulated infrared topic

        -- ************************ subscriber  ***************************

        bodyOutputSub=simROS.subscribe('/sim_body_topic','std_msgs/Float32MultiArray', 'body_cb') -- body signal
        CPGOutputSub=simROS.subscribe('/cpg_topic','std_msgs/Float32MultiArray', 'cpg_cb') -- cpg signal
        efferenceSub=simROS.subscribe('/efference_topic','std_msgs/Float32MultiArray', 'efference_cb') -- efference topic
        MOTOROutputSub=simROS.subscribe('/motor_topic','std_msgs/Float32MultiArray', 'motor_cb') -- motor topic
        simMOTOROutputSub=simROS.subscribe('/sim_motor_topic','std_msgs/Float32MultiArray', 'sim_motor_cb') -- motor topic
        indicatorSub=simROS.subscribe('/indicator_topic','std_msgs/Float32MultiArray', 'indicator_cb') -- performance measurement
        stretchingSub=simROS.subscribe('/stretching_topic','std_msgs/Float32MultiArray', 'stretching_cb') -- stretching signal
        stretchingSub=simROS.subscribe('/stretching_conc_topic','std_msgs/Float32MultiArray', 'stretching_conc_cb') -- stretching concentration
        confidenceSub=simROS.subscribe('/confidenceInspired_topic','std_msgs/Float32MultiArray', 'confidence_cb') -- confidence inspired concentration
        inclinationConcSub=simROS.subscribe('/inclination_conc_topic','std_msgs/Float32MultiArray', 'incinationConc_cb') -- body inclination topic
        realForceSub=simROS.subscribe('/real_force_topic','std_msgs/Float32MultiArray', 'real_force_cb') -- confidence inspired concentration
        realBodyAngSub=simROS.subscribe('/real_bodyAng_topic','std_msgs/Float32MultiArray', 'real_bodyAng_cb') -- confidence inspired concentration
        realIMUSub=simROS.subscribe('/real_imu_topic','std_msgs/Float32MultiArray', 'real_imu_cb') -- confidence inspired concentration


        -- Start the client application (c++ node)

        -- node to run during simulation start

        local rosnode = {'gecko_controller'}
        for i = 1,table.getn(rosnode),1 do
            result=sim.launchExecutable(simGetStringParameter(sim_stringparam_scene_path) .. '/../../../Projects/Slalom/catkin_ws/src/'..rosnode[i]..'/bin/'..rosnode[i],'/cpg_topic',0)
        end

        if (result==false) then
            sim.displayDialog('Error','External ROS-Node not found',sim.dlgstyle_ok,false,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
        end
    end
end


--[[
Actuation: This part will be executed in each simulation step
--]]
function sysCall_actuation()

    t = sim.getSimulationTime()

    for i=1,4,1 do --leg

        for j=1,4,1 do -- joint
            -- start simulation after 10 simulated second
            if(t > 10) then
                sim.setJointPosition(jointHandle[i][j], sim_motorSignal[4*(i-1)+j])

                -- body
                sim.setJointPosition(jointB1, bodySignal[1])
                sim.setJointPosition(jointB2, bodySignal[2])
                sim.setJointPosition(jointB3, bodySignal[3]) 

            end
        end

        -- get force
        forceData[i] = -10
        if(t > 10) then
            _,force,torque = simReadForceSensor(forceHandle[i])
            forceData[i] = force[3]
        end

        -- get distance
        res,dist = simCheckDistance(floor,footHandle[i],1)
        distance[i] = math.abs(dist[3]-dist[6])

    end

    -- Read the proximity sensor

    IRL_dist=sim.readProximitySensor(irlSensor)
    IRR_dist=sim.readProximitySensor(irrSensor)

    -- If we detected something, we set the backward mode:
    if (IRL_dist>0) then backUntilTime=sim.getSimulationTime()+4 end 
    if (IRR_dist>0) then backUntilTime=sim.getSimulationTime()+4 end 

    -- publishing topic

    forceTopic['data'] = forceData
    imuTopic['data'] = {accel[1],accel[2],accel[3],gyroData[1],gyroData[2],gyroData[3]}
    simTimeTopic['data'] = {t}
    simBodyAngTopic['data'] = {bodyAng[1],bodyAng[2],bodyAng[3]}
    simInfraredTopic['data'] = {IRL_dist,IRR_dist}
    

    simROS.publish(forcePublisher,forceTopic)
    simROS.publish(imuPublisher,imuTopic)
    simROS.publish(simTimePublisher,simTimeTopic)
    simROS.publish(simBodyAngPublisher,simBodyAngTopic)
    simROS.publish(simInfraredPublisher,simInfraredTopic)


    -- calculate walking distance
    local current_body_position = simGetObjectPosition(body,floor)

    walking_dist = math.abs(current_body_position[2]-body_init_posi[2])

    -- stop the simulation
    --time = simGetStringParameter
    --if(0) then
        --sim.stopSimulation()
    --end
    if(t < 10) then
        body_init_posi = simGetObjectPosition(body,floor)
    end

end

function sysCall_sensing()

    -- get simulated angluar vilocity and acceleration from imu

    local transformationMatrix=sim.getObjectMatrix(refG,-1)
    local oldInverse=simGetInvertedMatrix(oldTransformationMatrix)
    local m=sim.multiplyMatrices(oldInverse,transformationMatrix)
    local euler=sim.getEulerAnglesFromMatrix(m)
    local currentTime=sim.getSimulationTime()
    local ang=sim.getEulerAnglesFromMatrix(transformationMatrix)

    local dt=currentTime-lastTime
    if (dt~=0) then
        if(realRobot ~= 1) then
            gyroData[1]=euler[1]/dt
            gyroData[2]=euler[2]/dt
            gyroData[3]=euler[3]/dt
        end
    end
    bodyAng[1] = ang[1]*180/3.14159
    bodyAng[2] = ang[2]*180/3.14159
    bodyAng[3] = ang[3]*180/3.14159
    oldTransformationMatrix=sim.copyMatrix(transformationMatrix)
    lastTime=currentTime

    _,accForce=sim.readForceSensor(accSensor)


    if (realRobot ~= 1) then
        accel={accForce[1]/mass,accForce[2]/mass,accForce[3]/mass}
    else
    end

    simSetGraphUserData(graph,"gyrox",(gyroData[1]))
    simSetGraphUserData(graph,"gyroy",(gyroData[2]))
    simSetGraphUserData(graph,"gyroz",(gyroData[3]))
    simSetGraphUserData(graph,"accex",(accel[1]))
    simSetGraphUserData(graph,"accey",(accel[2]))
    simSetGraphUserData(graph,"accez",(accel[3]))

    simSetGraphUserData(graph,"irLeft",(IRL_dist))
    simSetGraphUserData(graph,"irRight",(IRR_dist))


    if (realRobot) then
        simSetGraphUserData(graph,"f_lf",(real_forceData[1]))
        simSetGraphUserData(graph,"f_rf",(real_forceData[2]))
        simSetGraphUserData(graph,"f_rh",(real_forceData[3]))
        simSetGraphUserData(graph,"f_lh",(real_forceData[4]))

        simSetGraphUserData(graph,"body_x",(real_bodyAng[1]))
        simSetGraphUserData(graph,"body_y",(real_bodyAng[2]))
    else
        simSetGraphUserData(graph,"f_lf",(forceData[1]))
        simSetGraphUserData(graph,"f_rf",(forceData[2]))
        simSetGraphUserData(graph,"f_rh",(forceData[3]))
        simSetGraphUserData(graph,"f_lh",(forceData[4]))
        simSetGraphUserData(graph,"body_x",(bodyAng[1]))
        simSetGraphUserData(graph,"body_y",(bodyAng[2]))
    end
    simSetGraphUserData(graph,"effe1",(effeSignal[1]))
    simSetGraphUserData(graph,"effe2",(effeSignal[2]))
    simSetGraphUserData(graph,"effe3",(effeSignal[3]))
    simSetGraphUserData(graph,"effe4",(effeSignal[4]))

    simSetGraphUserData(graph,"stab",stab)
    simSetGraphUserData(graph,"harm",harm)

    simSetGraphUserData(graph,"Hs_conc1",stretchingConc[1])
    simSetGraphUserData(graph,"Hs_conc2",stretchingConc[2])
    simSetGraphUserData(graph,"Hs_conc3",stretchingConc[3])
    simSetGraphUserData(graph,"Hs_conc4",stretchingConc[4])
    simSetGraphUserData(graph,"Hs_sig1",stretchingSig[1])
    simSetGraphUserData(graph,"Hs_sig2",stretchingSig[2])
    simSetGraphUserData(graph,"Hs_sig3",stretchingSig[3])
    simSetGraphUserData(graph,"Hs_sig4",stretchingSig[4])
    simSetGraphUserData(graph,"conf_conc",confidenceConc)
    simSetGraphUserData(graph,"H_inc",incConc)

    simSetGraphUserData(graph,"cpg_sig1",cpgSignal[1])
    simSetGraphUserData(graph,"cpg_sig2",cpgSignal[2])
    simSetGraphUserData(graph,"vrn_sig1",cpgSignal[3])
    simSetGraphUserData(graph,"vrn_sig2",cpgSignal[4])

    simSetGraphUserData(g_test,"t1",real_forceData[1])
    simSetGraphUserData(g_test,"t2",real_forceData[2])
    simSetGraphUserData(g_test,"t3",real_bodyAng[2])
    simSetGraphUserData(g_test,"t4",real_forceData[4])

end

function sysCall_cleanup()
    -- do some clean-up here
    walking_dist = math.abs(simGetObjectPosition(body,floor)[2]-body_init_posi[2])
    t = sim.getSimulationTime()
    print("average speed of this test case is " .. walking_dist/(t-9) .. "m/s")

    simROS.shutdownSubscriber(bodyOutputSub)
    simROS.shutdownSubscriber(CPGOutputSub)
    simROS.shutdownSubscriber(MOTOROutputSub)
    simROS.shutdownSubscriber(simMOTOROutputSub)
    simROS.shutdownSubscriber(indicatorSub)
    simROS.shutdownSubscriber(stretchingSub)
    simROS.shutdownSubscriber(confidenceSub)
    simROS.shutdownSubscriber(inclinationConcSub)
    simROS.shutdownSubscriber(realForceSub)
    simROS.shutdownSubscriber(realIMUSub)
end
