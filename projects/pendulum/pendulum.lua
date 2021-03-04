--[[
Callback function for reciving motor positions
from ROS controller node
--]]
function setMotorPositions_cb(msg)
    data = msg.data
    if (data[1] < 0) then
        simSetJointTargetVelocity(cart_joint, -999999999999)   -- Unreacheable low number
    elseif (data[1] == 0) then
        simSetJointTargetVelocity(cart_joint, 0)
    else
        simSetJointTargetVelocity(cart_joint, 999999999999)    -- Unreacheable high number
    end

    print('\nForce:\n')
    print(data[1])

    if(simGetSimulationTime()>-1) then
        simSetJointForce(cart_joint,math.abs(data[1]))
    else
        simSetJointForce(cart_joint,math.abs(0))
    end
end

--[[
Callback function for displaying data on a graph
resived from ROS controller node
--]]
function graph_cb(msg)
    data = msg.data

    simSetGraphUserData(graphHandle,"O1",data[1])
    simSetGraphUserData(graphHandle,"O2",data[2])
    simSetGraphUserData(graphHandle,"O3",data[3])
end

--[[
Function for handling slider changes on the QT UI
--]]
function sliderChange(ui,id,newVal)
    if id == 10+0 then
        newVal = newVal/1000
        simUI.setLabelText(ui,1000+0,'Value: '..newVal)
        testParameters[1] = newVal
    elseif id == 10+1 then
        simUI.setLabelText(ui,1000+1,'Value: '..newVal)
        testParameters[2] = newVal
    elseif id == 10+2 then
        newVal = newVal/1000
        simUI.setLabelText(ui,1000+2,'Value: '..newVal)
        testParameters[3] = newVal
    elseif id == 10+3 then
        newVal = newVal/1000
        simUI.setLabelText(ui,1000+3,'Value: '..newVal)
        testParameters[4] = newVal
    end
end

--[[
Initialization: Called once at the start of a simulation
--]]
function sysCall_init()

    -- User Interface setup
    xml = [[ <ui closeable="false" resizable="true" style="plastique" title="Legged-Robot UI">
    <label text="Cart Acc." wordwrap="true" />
    <label text="Value: 0" id="1000" wordwrap="true" />
    <hslider id="10" tick-position="both-sides" tick-interval="1" minimum="-10000" maximum="10000" onchange="sliderChange" style="plastique" />

    <label text="Test Parameter" wordwrap="true" />
    <label text="Value: 0" id="1001" wordwrap="true" />
    <hslider id="11" tick-position="both-sides" tick-interval="1" minimum="0" maximum="50" onchange="sliderChange" style="plastique" />

    <label text="Test Parameter" wordwrap="true" />
    <label text="Value: 0" id="1002" wordwrap="true" />
    <hslider id="12" tick-position="both-sides" tick-interval="1" minimum="0" maximum="1500" onchange="sliderChange" style="plastique" />

    <label text="Test Parameter" wordwrap="true" />
    <label text="Value: 0" id="1003" wordwrap="true" />
    <hslider id="13" tick-position="both-sides" tick-interval="1" minimum="0" maximum="8000" onchange="sliderChange" style="plastique" />
    </ui> ]]

    -- Create User Interface
    ui=simUI.create(xml)

    -- Initialize sliders and testparameters (these are shared with the ROS node)
    testParameters = {0, 10, 0.14, 6}
    simExtCustomUI_setSliderValue(ui,10,testParameters[1]*1000)
    sliderChange(ui,10,testParameters[1]*1000)
    simExtCustomUI_setSliderValue(ui,11,testParameters[2])
    sliderChange(ui,11,testParameters[2])
    simExtCustomUI_setSliderValue(ui,12,testParameters[3]*1000)
    sliderChange(ui,12,testParameters[3]*1000)
    simExtCustomUI_setSliderValue(ui,13,testParameters[4]*1000)
    sliderChange(ui,13,testParameters[4]*1000)

    -- Set position of the User Interface
    x, y=simExtCustomUI_getPosition(ui)
    simExtCustomUI_setPosition(ui, x+500, y+200, true)

    -- Create all handles
    pendulumHandle=sim.getObjectAssociatedWithScript(sim.handle_self)
    cart_joint=sim.getObjectHandle("cart_joint")
    base_joint=sim.getObjectHandle("base_joint")
    arm_joint=sim.getObjectHandle("arm_joint")
    graphHandle=sim.getObjectHandle("rosGraph")

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
        local MotorTopicName='MotorPositions'
        local simulationTimeTopicName='simTime'
        local startSimulationName='startSimulation'
        local pauseSimulationName='pauseSimulation'
        local stopSimulationName='stopSimulation'
        local enableSyncModeName='enableSyncMode'
        local triggerNextStepName='pauseSimulation'
        local simulationStepDoneName='simulationStepDone'
        local simulationStateName='simulationState'
        local terminateControllerName='terminateController'
        local CPGOutputName='CPGOutput' -- not used
        local jointPositionsName='jointPositions'
        local testParametersName='testParameters'

        local SimulationType='legged'

        -- Create the subscribers
        MotorSub=simROS.subscribe('/'..MotorTopicName,'std_msgs/Float32MultiArray','setMotorPositions_cb')
        CPGOutputSub=simROS.subscribe('/'..CPGOutputName,'std_msgs/Float32MultiArray', 'graph_cb')

        -- Create the publishers
        terminateControllerPub=simROS.advertise('/'..terminateControllerName,'std_msgs/Bool')
        jointPositionsPub=simROS.advertise('/'..jointPositionsName,'std_msgs/Float32MultiArray')
        clockPub=simROS.advertise('/clock','rosgraph_msgs/Clock')
        testParametersPub=simROS.advertise('/'..testParametersName,'std_msgs/Float32MultiArray')

        -- Start the client application (c++ node)
        result=sim.launchExecutable(simGetStringParameter(sim_stringparam_scene_path) .. '/../../../projects/pendulum/catkin_ws/src/pendulum_controller/bin/pendulum_controller', MotorTopicName.." "..simulationTimeTopicName.." "..terminateControllerName.." "..startSimulationName.." "..pauseSimulationName.." "..stopSimulationName.." "..enableSyncModeName.." "..triggerNextStepName.." "..simulationStepDoneName.." "..simulationStateName.." "..CPGOutputName.." "..jointPositionsName.." "..testParametersName.." "..SimulationType,0)
        if (result==false) then
            sim.displayDialog('Error','External ROS-Node not found',sim.dlgstyle_ok,false,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
        end
    end

    -- Publish Initial Velocity and Position
    result,velC=simGetObjectFloatParameter(cart_joint,2012)
    result,velB=simGetObjectFloatParameter(base_joint,2012)
    result,velA=simGetObjectFloatParameter(arm_joint,2012)

    posC=simGetJointPosition(cart_joint)
    posB=simGetJointPosition(base_joint)
    posA=simGetJointPosition(arm_joint)

    position_array={posC, posB, posA, velC, velB, velA}

    simROS.publish(jointPositionsPub,{data=position_array})
end


--[[
Actuation: This part will be executed in each simulation step
--]]
function sysCall_actuation()

    -- Publish Clock for Sync.
    simROS.publish(clockPub,{clock=simGetSimulationTime()})

end

--[[
Sensing: This part will be executed in each simulation step
--]]
function sysCall_sensing()

    -- Publish current Velocity and Position
    result,velC=simGetObjectFloatParameter(cart_joint,2012)
    result,velB=simGetObjectFloatParameter(base_joint,2012)
    result,velA=simGetObjectFloatParameter(arm_joint,2012)

    posC=simGetJointPosition(cart_joint)
    posB=simGetJointPosition(base_joint)
    posA=simGetJointPosition(arm_joint)

    position_array={posC, posB, posA, velC, velB, velA}

    simROS.publish(jointPositionsPub,{data=position_array})

    -- Publish test parameters from the QT UI
    simROS.publish(testParametersPub,{data=testParameters})

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

    -- Close UI
    simUI.destroy(ui)

    -- Clode ROS related stuff
    if not pluginNotFound then
        -- Send termination signal to external ROS nodes
        simROS.publish(terminateControllerPub,{data=true})

        -- Terminate remaining local notes
        simROS.shutdownSubscriber(CPGOutputSub)
        simROS.shutdownSubscriber(MotorSub)
        simROS.shutdownPublisher(clockPub)
        simROS.shutdownPublisher(jointPositionsPub)
        simROS.shutdownPublisher(testParametersPub)
        simROS.shutdownPublisher(terminateControllerPub)
    end
end
