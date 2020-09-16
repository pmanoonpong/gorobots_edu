--[[
Author: Sun Tao
Email: suntao.hn@qq.com
Porject: NEUTRON
Callback function for reciving motor positions
from ROS controller node

--]]
-----------------------**********************--------------------------
function buttonClick_OpenMotor()
	sim.addStatusbarMessage('You clicked the button of OpenMotor')
	buttonClickState_OpenMotor = true
end

--[[
Function for handling slider changes on the QT UI
--]]
function sliderChange(ui,id,newVal)
    sliderChangeState=true
    if id<21
        then
        newVal = newVal/200.0
        simUI.setLabelText(ui,990+id,'Value: '..newVal)
        controlParameters[id-9] = newVal
    else
        simUI.setLabelText(ui,990+id,'Value: '..newVal)
        controlParameters[id-9] = newVal
    end
end


function setMotorPositions()
	data={}
--	if (buttonClickState_OpenMotor == true)
--	 then
   		for i = 1, #joint_array do
 	 		data[i] = CPGData[i] 
            --+ reflexData[i]
 	 	    --	data[i] = reflexData[i]
		end

        for i = 1, #joint_array do
			simSetJointTargetPosition(joint_array[i], data[i])	
        end
--	end
	
end


function setCPGMotorData(msg)
	CPGData=msg.data
end

function setReflexMotorData(msg)
	reflexData=msg.data
end

function graph_cb(msg)
    data = msg.data
    simSetGraphUserData(graphHandleRF,"CPGN0",data[1])
    simSetGraphUserData(graphHandleRF,"CPGN1",data[2])
    simSetGraphUserData(graphHandleRF,"PCPGN0",data[3])
    simSetGraphUserData(graphHandleRF,"PCPGN1",data[4])
    simSetGraphUserData(graphHandleRF,"PSN10",data[5])
    simSetGraphUserData(graphHandleRF,"PSN11",data[6])
    simSetGraphUserData(graphHandleRF,"VRNHip",data[7])
    simSetGraphUserData(graphHandleRF,"VRNKnee",data[8])
	simSetGraphUserData(graphHandleRF,"ReflexOut0",data[9])
    simSetGraphUserData(graphHandleRF,"ReflexOut1",data[10])
    simSetGraphUserData(graphHandleRF,"ReflexOut2",data[11])
    simSetGraphUserData(graphHandleRF,"GRF",data[12])
--[[
	simSetGraphUserData(graphHandleRF,"PMN0",data[13])
    simSetGraphUserData(graphHandleRF,"PMN1",data[14])
    simSetGraphUserData(graphHandleRF,"PMN2",data[15])
	simSetGraphUserData(graphHandleRF,"Jaf0",data[16])
    simSetGraphUserData(graphHandleRF,"Jaf1",data[17])
    simSetGraphUserData(graphHandleRF,"Jaf2",data[18])
--]]
    simSetGraphUserData(graphHandleRH,"CPGN0",data[19])
    simSetGraphUserData(graphHandleRH,"CPGN1",data[20])
    simSetGraphUserData(graphHandleRH,"PCPGN0",data[21])
    simSetGraphUserData(graphHandleRH,"PCPGN1",data[22])
    simSetGraphUserData(graphHandleRH,"PSN10",data[23])
    simSetGraphUserData(graphHandleRH,"PSN11",data[24])
    simSetGraphUserData(graphHandleRH,"VRNHip",data[25])
    simSetGraphUserData(graphHandleRH,"VRNKnee",data[26])
	simSetGraphUserData(graphHandleRH,"ReflexOut0",data[27])
    simSetGraphUserData(graphHandleRH,"ReflexOut1",data[28])
    simSetGraphUserData(graphHandleRH,"ReflexOut2",data[29])
    simSetGraphUserData(graphHandleRH,"GRF",data[30])
--[[
	simSetGraphUserData(graphHandleRH,"PMN0",data[31])
    simSetGraphUserData(graphHandleRH,"PMN1",data[32])
    simSetGraphUserData(graphHandleRH,"PMN2",data[33])
	simSetGraphUserData(graphHandleRH,"Jaf0",data[34])
    simSetGraphUserData(graphHandleRH,"Jaf1",data[35])
    simSetGraphUserData(graphHandleRH,"Jaf2",data[36])
--]]

    simSetGraphUserData(graphHandleLF,"CPGN0",data[37])
    simSetGraphUserData(graphHandleLF,"CPGN1",data[38])
    simSetGraphUserData(graphHandleLF,"PCPGN0",data[39])
    simSetGraphUserData(graphHandleLF,"PCPGN1",data[40])
    simSetGraphUserData(graphHandleLF,"PSN10",data[41])
    simSetGraphUserData(graphHandleLF,"PSN11",data[42])
    simSetGraphUserData(graphHandleLF,"VRNHip",data[43])
    simSetGraphUserData(graphHandleLF,"VRNKnee",data[44])
	simSetGraphUserData(graphHandleLF,"ReflexOut0",data[45])
    simSetGraphUserData(graphHandleLF,"ReflexOut1",data[46])
    simSetGraphUserData(graphHandleLF,"ReflexOut2",data[47])
    simSetGraphUserData(graphHandleLF,"GRF",data[48])
--[[
	simSetGraphUserData(graphHandleLF,"PMN0",data[49])
    simSetGraphUserData(graphHandleLF,"PMN1",data[50])
    simSetGraphUserData(graphHandleLF,"PMN2",data[51])
	simSetGraphUserData(graphHandleLF,"Jaf0",data[52])
    simSetGraphUserData(graphHandleLF,"Jaf1",data[53])
    simSetGraphUserData(graphHandleLF,"Jaf2",data[54])
--]]

    simSetGraphUserData(graphHandleLH,"CPGN0",data[55])
    simSetGraphUserData(graphHandleLH,"CPGN1",data[56])
    simSetGraphUserData(graphHandleLH,"PCPGN0",data[57])
    simSetGraphUserData(graphHandleLH,"PCPGN1",data[58])
    simSetGraphUserData(graphHandleLH,"PSN10",data[59])
    simSetGraphUserData(graphHandleLH,"PSN11",data[60])
    simSetGraphUserData(graphHandleLH,"VRNHip",data[61])
    simSetGraphUserData(graphHandleLH,"VRNKnee",data[62])
	simSetGraphUserData(graphHandleLH,"ReflexOut0",data[63])
    simSetGraphUserData(graphHandleLH,"ReflexOut1",data[64])
    simSetGraphUserData(graphHandleLH,"ReflexOut2",data[65])
    simSetGraphUserData(graphHandleLH,"GRF",data[66])
--[[
	simSetGraphUserData(graphHandleLH,"PMN0",data[67])
    simSetGraphUserData(graphHandleLH,"PMN1",data[68])
    simSetGraphUserData(graphHandleLH,"PMN2",data[69])
	simSetGraphUserData(graphHandleLH,"Jaf0",data[70])
    simSetGraphUserData(graphHandleLH,"Jaf1",data[71])
    simSetGraphUserData(graphHandleLH,"Jaf2",data[72])
--]]
end

--[[
Callback function for displaying data on a graph
resived from ROS controller node
--]]

function initParametersSlide()
 
	 -- User Interface setup
        
    xml = [[ <ui closeable="false" resizable="true" style="plastique" title="PSN-VRN ">
    <label text="Psn_L:" wordwrap="true" />
    <label text="Value: 0" id="1000" wordwrap="true" />
    <hslider id="10" tick-position="both-sides" tick-interval="1" minimum="-200" maximum="200" onchange="sliderChange" style="plastique" />

    <label text="VRNHip_L" wordwrap="true" />
    <label text="Value: 0" id="1001" wordwrap="true" />
    <hslider id="11" tick-position="both-sides" tick-interval="1" minimum="-200" maximum="200" onchange="sliderChange" style="plastique" />

    <label text="VRNKnee_L" wordwrap="true" />
    <label text="Value: 0" id="1002" wordwrap="true" />
    <hslider id="12" tick-position="both-sides" tick-interval="1" minimum="-200" maximum="200" onchange="sliderChange" style="plastique" />

    <label text="Psn_R" wordwrap="true" />
    <label text="Value: 0" id="1003" wordwrap="true" />
    <hslider id="13" tick-position="both-sides" tick-interval="1" minimum="-200" maximum="200" onchange="sliderChange" style="plastique" />
    
    <label text="VRNHip_R" wordwrap="true" />
    <label text="Value: 0" id="1004" wordwrap="true" />
    <hslider id="14" tick-position="both-sides" tick-interval="1" minimum="-200" maximum="200" onchange="sliderChange" style="plastique" />
    
    <label text="VRNKnee_R" wordwrap="true" />
    <label text="Value: 0" id="1005" wordwrap="true" />
    <hslider id="15" tick-position="both-sides" tick-interval="1" minimum="-200" maximum="200" onchange="sliderChange" style="plastique" />

	</ui> ]]
    -- Create User Interface
    uiPSNVRN=simUI.create(xml)
    xml= [[ <ui closeable="false" resizable="true" style="plastique" title="MNBias">
    <label text="MNBias1" wordwrap="true" />
    <label text="Value: 0" id="1006" wordwrap="true" />
    <hslider id="16" tick-position="both-sides" tick-interval="1" minimum="-200" maximum="200" onchange="sliderChange" style="plastique" />

    <label text="MNBias2" wordwrap="true" />
    <label text="Value: 0" id="1007" wordwrap="true" />
    <hslider id="17" tick-position="both-sides" tick-interval="1" minimum="-750" maximum="600" onchange="sliderChange" style="plastique" />

    <label text="MNBias3" wordwrap="true" />
    <label text="Value: 0" id="1008" wordwrap="true" />
    <hslider id="18" tick-position="both-sides" tick-interval="1" minimum="-750" maximum="600" onchange="sliderChange" style="plastique" />
    </ui> ]]
    uiMNBias=simUI.create(xml)
    xml = [[ <ui closeable="false" resizable="true" style="plastique" title="CPG-PCPG">
    <label text="CPGMi" wordwrap="true" />
    <label text="Value: 0" id="1009" wordwrap="true" />
    <hslider id="19" tick-position="both-sides" tick-interval="1" minimum="-200" maximum="200" onchange="sliderChange" style="plastique" />

    <label text="PCPGBeta" wordwrap="true" />
    <label text="Value: 0" id="1010" wordwrap="true" />
    <hslider id="20" tick-position="both-sides" tick-interval="1" minimum="-200" maximum="200" onchange="sliderChange" style="plastique" />

    <label text="CPGSType" wordwrap="true" />
    <label text="Value: 0" id="1011" wordwrap="true" />
    <hslider id="21" tick-position="both-sides" tick-interval="1" minimum="0" maximum="7" onchange="sliderChange" style="plastique" />

    <button text='Drive the Motor' on-click = "buttonClick_OpenMotor" />
    </ui> ]]
    uiCPGPCPG=simUI.create(xml)
	buttonClickState_OpenMotor =false
    sliderChangeState= false

    controlParameters={ 0.0,0.04,0.04, 0.0,0.04,0.04, 0.0,-0.06,-0.06,  0.1,0.75,5}
    -- Initialize sliders and testparameters (these are shared with the ROS node)
    for i=1,6 do
    simExtCustomUI_setSliderValue(uiPSNVRN,9+i,controlParameters[i]*200.0)
    sliderChange(uiPSNVRN,9+i,controlParameters[i]*200.0)
    end
    for i=1,3 do
    simExtCustomUI_setSliderValue(uiMNBias,15+i,controlParameters[6+i]*200.0)
    sliderChange(uiMNBias,15+i,controlParameters[6+i]*200.0)
    end
    for i=1,2 do
    simExtCustomUI_setSliderValue(uiCPGPCPG,18+i,controlParameters[9+i]*200.0)
    sliderChange(uiCPGPCPG,18+i,controlParameters[9+i]*200.0)
    end
    simExtCustomUI_setSliderValue(uiCPGPCPG,18+3,controlParameters[9+3])
    sliderChange(uiCPGPCPG,18+3,controlParameters[9+3])
    -- Set position of the User Interface
    x, y=simExtCustomUI_getPosition(uiPSNVRN)
    simExtCustomUI_setPosition(uiPSNVRN, x+800, y-300, true)
    x, y=simExtCustomUI_getPosition(uiMNBias)
    simExtCustomUI_setPosition(uiMNBias, x+600, y-300, true)
    x, y=simExtCustomUI_getPosition(uiCPGPCPG)
    simExtCustomUI_setPosition(uiCPGPCPG, x+600, y-300, true)

end

    --[[
        Initialization: Called once at the start of a simulation
    --]]
function sysCall_init()
    -- Create all handles
    lilibotHandle=sim.getObjectAssociatedWithScript(sim.handle_self)

    FR_joint_1=sim.getObjectHandle("RF_joint1")
    FR_joint_2=sim.getObjectHandle("RF_joint2")
    FR_joint_3=sim.getObjectHandle("RF_joint3")

    RR_joint_1=sim.getObjectHandle("RH_joint1")
    RR_joint_2=sim.getObjectHandle("RH_joint2")
    RR_joint_3=sim.getObjectHandle("RH_joint3")

    FL_joint_1=sim.getObjectHandle("LF_joint1")
    FL_joint_2=sim.getObjectHandle("LF_joint2")
    FL_joint_3=sim.getObjectHandle("LF_joint3")

    RL_joint_1=sim.getObjectHandle("LH_joint1")
    RL_joint_2=sim.getObjectHandle("LH_joint2")
    RL_joint_3=sim.getObjectHandle("LH_joint3")

    joint_array={FR_joint_1, FR_joint_2, FR_joint_3, RR_joint_1, RR_joint_2, RR_joint_3, FL_joint_1, FL_joint_2, FL_joint_3, RL_joint_1, RL_joint_2, RL_joint_3}
    FR_foot_sensor=sim.getObjectHandle("RF_foot_sensor")
    FL_foot_sensor=sim.getObjectHandle("LF_foot_sensor")
    RR_foot_sensor=sim.getObjectHandle("RH_foot_sensor")
    RL_foot_sensor=sim.getObjectHandle("LH_foot_sensor")
    foot_sensor_array = { FR_foot_sensor, RR_foot_sensor, FL_foot_sensor,RL_foot_sensor }
    body = sim.getObjectHandle("body")
    for i = 1, #joint_array do
    print(sim.getObjectName(joint_array[i]))
    end

    for i = 1, #foot_sensor_array do
    print(sim.getObjectName(foot_sensor_array[i]))
    end

    graphHandleRF=sim.getObjectHandle("InspectableRF")
    graphHandleRH=sim.getObjectHandle("InspectableRH")
    graphHandleLF=sim.getObjectHandle("InspectableLF")
    graphHandleLH=sim.getObjectHandle("InspectableLH")
    -- Check if the required ROS plugin is loaded
    moduleName=0
    moduleVersion=0
    index=0
    pluginNotFound=true
    while moduleName do
        moduleName,moduleVersion=sim.getModuleName(index)
    if (moduleName=='RosInterface') then
    pluginNotFound=false
    end
    index=index+1
    end
    if (pluginNotFound) then
    sim.displayDialog('Error','The RosInterface was not found.',sim.dlgstyle_ok,false,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
    end

    -- If found then start the subscribers and publishers
    if (not pluginNotFound) then
        local cpgMotorName='motorValue'
        local simulationTimeName='simTime'
        local terminateControllerName='terminateController'
        local startSimulationName='startSimulation'
        local pauseSimulationName='pauseSimulation'
        local stopSimulationName='stopSimulation'
        local enableSyncModeName='enableSyncMode'
        local triggerNextStepName='pauseSimulation'
        local simulationStepDoneName='simulationStepDone'
        local simulationStateName='simulationState'
        local neuroNetworkOutputName='neuroNetworkOutput' 
        local sensorValueName='sensorValue'
		local reflexMotorName= 'reflexValue'

        -- Create the subscribers
        cpgValueSub=simROS.subscribe('/'..cpgMotorName,'std_msgs/Float32MultiArray','setCPGMotorData')
        neuroNetworkOutputSub=simROS.subscribe('/'..neuroNetworkOutputName,'std_msgs/Float32MultiArray', 'graph_cb')
        -- Create the subscribers
        reflexValueSub=simROS.subscribe('/'..reflexMotorName,'std_msgs/Float32MultiArray','setReflexMotorData')

        -- Create the publishers
        terminateControllerPub=simROS.advertise('/'..terminateControllerName,'std_msgs/Bool')
        sensorValuePub=simROS.advertise('/'..sensorValueName,'std_msgs/Float32MultiArray')
        simulationTimePub=simROS.advertise('/'..simulationTimeName,'rosgraph_msgs/Clock')
        -- Start rosnode
        result=sim.launchExecutable(simGetStringParameter(sim_stringparam_scene_path) .. '/start_lilibot_vrep',0)
        if (result==false) then
            sim.displayDialog('Error','External reflex ROS-Node not found',sim.dlgstyle_ok,false,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
        end
           

    end


   	 -- Publish Initial Velocity and Position
	vel , pos = {}, {}
	sensor_array = {}
	for i = 1, #joint_array do
	pos[i] = simGetJointPosition(joint_array[i])
	sensor_array[i] = pos[i]
	end
    simROS.publish(sensorValuePub,{data=sensor_array})
    simROS.publish(simulationTimePub,{clock=simGetSimulationTime()})

	--Subscriber initial  data
	CPGData={}
    reflexData={}
	
    for i = 1, #joint_array do
    CPGData[i] = 0.0
    reflexData[i] = 0.0
    end

    -- init slide for adjust paramterts

   -- initParametersSlide() 

end


    --[[
        Actuation: This part will be executed in each simulation step
    --]]
function sysCall_actuation()

    -- Publish Clock for Sync.
    setMotorPositions()
end

    --[[
        Sensing: This part will be executed in each simulation step
    --]]
function sysCall_sensing()

    -- Publish sensor values----------------	

    -- Joint Velocity and Position, attitude angles of body ,and Foot contact force
    for i = 1, #joint_array do
    pos[i] = simGetJointPosition(joint_array[i])
    sensor_array[i] = pos[i]
    end	

    for i = 1, #foot_sensor_array do
    reuslt,force,torque = sim.readForceSensor(foot_sensor_array[i])
    sensor_array[i + #joint_array] = force[3]/10.0 --+ ((force[3]/10.0 > 0.1) and 0.5*math.random() or 0.0) --let the value into [0,1] --force[3]/10.0 
    end
    -- please don't change this value,

    eularAngles=sim.getObjectOrientation(body,-1)
    for i = 1, #eularAngles do
    sensor_array[i + #joint_array + #foot_sensor_array]= eularAngles[i]
    end

    for i = 1, #foot_sensor_array do
    reuslt,force,torque = sim.readForceSensor(foot_sensor_array[i])
    sensor_array[i + #joint_array + #foot_sensor_array + #eularAngles] = force[1]/10.0	--let the value into [0,1]
    end												-- please don't change this value,
    simROS.publish(sensorValuePub,{data=sensor_array})

    -- publish clock
    simROS.publish(simulationTimePub,{clock=simGetSimulationTime()})

    -- set the parameters use parameters service
    if(sliderChangeState)
        then
        simROS.setParamDouble("PSN_L1",controlParameters[1]);
        simROS.setParamDouble("VRN_hip_L1",controlParameters[2]);
        simROS.setParamDouble("VRN_knee_L1",controlParameters[3]);

        simROS.setParamDouble("PSN_L2",controlParameters[1]);
        simROS.setParamDouble("VRN_hip_L2",controlParameters[2]);
        simROS.setParamDouble("VRN_knee_L2",controlParameters[3]);

        simROS.setParamDouble("PSN_L3",controlParameters[4]);
        simROS.setParamDouble("VRN_hip_L3",controlParameters[5]);
        simROS.setParamDouble("VRN_knee_L3",controlParameters[6]);

        simROS.setParamDouble("PSN_L4",controlParameters[4]);
        simROS.setParamDouble("VRN_hip_L4",controlParameters[5]);
        simROS.setParamDouble("VRN_knee_L4",controlParameters[6]);

        simROS.setParamDouble("MNB1_L1",controlParameters[7]);
        simROS.setParamDouble("MNB2_L1",controlParameters[8]);
        simROS.setParamDouble("MNB3_L1",controlParameters[9]);

        simROS.setParamDouble("MNB1_L2",controlParameters[7]);
        simROS.setParamDouble("MNB2_L2",controlParameters[8]);
        simROS.setParamDouble("MNB3_L2",controlParameters[9]);

        simROS.setParamDouble("MNB1_L3",controlParameters[7]);
        simROS.setParamDouble("MNB2_L3",controlParameters[8]);
        simROS.setParamDouble("MNB3_L3",controlParameters[9]);

        simROS.setParamDouble("MNB1_L4",controlParameters[7]);
        simROS.setParamDouble("MNB2_L4",controlParameters[8]);
        simROS.setParamDouble("MNB3_L4",controlParameters[9]);

        simROS.setParamDouble("MI",controlParameters[10]);
        simROS.setParamDouble("PCPGBeta",controlParameters[11]);
        simROS.setParamDouble("CPGType",controlParameters[12]);
        print(controlParameters[12])
        sliderChangeState=false
    end
end

--[[
Sensing: This part will be executed one time just before a simulation ends
--]]
function sysCall_cleanup()
    
    -- Wait for the signal to arive at the nodes
    waitTimer=0
    while( waitTimer < 50000 ) do
        waitTimer = waitTimer+1
    end


    -- Clode ROS related stuff
    if not pluginNotFound then
        -- Send termination signal to external ROS nodes
        simROS.publish(terminateControllerPub,{data=true})

        -- Terminate subscriber
        simROS.shutdownSubscriber(neuroNetworkOutputSub)
        simROS.shutdownSubscriber(cpgValueSub)
        simROS.shutdownSubscriber(reflexValueSub)
		-- terminate publisher
        simROS.shutdownPublisher(simulationTimePub)
        simROS.shutdownPublisher(sensorValuePub)
        simROS.shutdownPublisher(terminateControllerPub)
    end
	-- close serial port -----
--	sim.serialClose(SerialPort)
end

