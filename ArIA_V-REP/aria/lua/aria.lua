-- Following function writes data to the socket (only single packet data for simplicity sake):
function writeSocketData(client,data)
   --print ("send "..#data.." bytes")
   local header=string.char(59,57,math.mod(#data,256),math.floor(#data/256),0,0)
   -- Packet header is (in this case): headerID (59,57), dataSize (WORD), packetsLeft (WORD) but not used here
   client:send(header..data)
end

-- Following function reads data from the socket (only single packet data for simplicity sake):
function readSocketData(client)
   -- Packet header is: headerID (59,57), dataSize (WORD), packetsLeft (WORD) but not used here
   local header=client:receive(6)
   if (header==nil) then
      return(nil) -- error
   end
   if (header:byte(1)==59)and(header:byte(2)==57) then
      local l=header:byte(3)+header:byte(4)*256
      --print ("receive "..l.." bytes")
      local v=client:receive(l)
      return(v)
   else
      return(nil) -- error
   end
end

-- Use sleeping function of socket library
function sleep(sec)
   socket.select(nil,nil,sec)
end 

-- Get cuurent time (in sec) 
function gettime()
   return socket.gettime()
end

-- Round with to given number of decimal places
function roundDecimal(num, numDecimalPlaces)
   local mult = 10^(numDecimalPlaces or 0)
   return math.floor(num * mult + 0.5) / mult
end

-- Simple round
function round(x)
  return x>=0 and math.floor(x+0.5) or math.ceil(x-0.5)
end

-- int() function
function signedFloorCeil(x)
  return x>=0 and math.floor(x) or math.ceil(x)
end

-- Get joint angular position (in degrees)
function getJointAngularPosition (handle)
    angle = simGetJointPosition(handle)
    angPos = angle*180.0/math.pi
    angPos = angPos + 180.0
    return angPos
end 

  
-- main simulation loop function (run every 50 ms)
function threadFunction()
   print ("sim state",simGetSimulationState())
   while (simGetSimulationState()~=sim_simulation_advancing_abouttostop) do
      --local t0 = clock()
      local t0 = gettime()
      local simTime = simGetSimulationTime()
      timeStamp = simTime..";"

      loc = simGetObjectPosition(rob,-1)
      orient = simGetObjectOrientation(rob,-1)
      --logMessage = timeStamp..clock()..";robot pose;"..loc[1]..";"..loc[2]..";"..orient[3]*180.0/math.pi
      logMessage = timeStamp..gettime()..";robot pose;"..loc[1]..";"..loc[2]..";"..loc[3]..";"..orient[3]*180.0/math.pi
      --print (logMessage)
      robX = loc[1]
      robY = loc[2]
      robZ = loc[3]
      robHead = orient[3]*180.0/math.pi
      local dataOut={robX,robY,robZ,robHead}
      for i=1,nJoints do
	 dataOut[#dataOut+1] = getJointAngularPosition(jointsId[i])
      end
      
      if not serverOn then
         print ("not connected")
         srv = assert(socket.bind('127.0.0.1',portNb))
         if (srv==nil) then
            print ("bad connect")
         else
            ip, port = srv:getsockname()
            print ("server ok at "..ip.." on port "..port)
            serverOn = true
            --srv:settimeout(connexionTimeout)
            print ("connexion granted !!! ")
         end
      end
      --print (serverOn)
      if serverOn then
         srv:settimeout(connexionTimeout)
         clt1 = srv:accept()
         if clt1 == nil then
            cntTimeout = cntTimeout + 1
             --print ("accept timeout")
             --serverOn = false
             --srv:close()
         else
            clt1:settimeout(connexionTimeout)
            dataIn = readSocketData(clt1)
            if dataIn ~= nil then
               --print (dataIn)
               targetCmd=simUnpackFloats(dataIn)            
	       for i=1,nJoints do
		  jointsCmd[i] = targetCmd[i]*degRad
		  if targetCmd[i] ~= 0 then
		     print ("Cmd"..(i+1).."="..targetCmd[i].." deg, "..jointsCmd[i].." rad")
		  end
		  simSetJointTargetPosition(jointsId[i],jointsCmd[i])
               end
               -- Pack the data as a string:srv:close()
               dataPacked=simPackFloats(dataOut)
               -- Send the data:
               writeSocketData(clt1,dataPacked)
               clt1:send(dataIn)
            else
               print ("no data")
            end
            clt1:close()
         end
      end

      simSwitchThread() -- This thread will resume just before the main script is called again
   end
end


-- Initialization:
print=printToConsole -- revert to use old way of print
print ("start init ...")
clock = os.clock

simSetThreadSwitchTiming(40) -- We wanna manually switch for synchronization purpose (and also not to waste processing time!)

-- Radian / degree convertion
radDeg = 180.0/math.pi
degRad = math.pi/180.0

rob = simGetObjectHandle("Body_Coxa")
LF_Coxa = 1
RF_Coxa = 2
LF_Femur = 3
RF_Femur = 4
LF_Tibia = 5
RF_Tibia = 6
LR_Coxa = 7
RR_Coxa = 8
LR_Femu = 9
RR_Femur = 10
LR_Tibia = 11
RR_Tibia = 12
LM_Coxa = 13
RM_Coxa = 14
LM_Femu = 15
RM_Femur = 16
LM_Tibia = 17
RM_Tibia = 18
jointsId={}
joints={"LF_Coxa","RF_Coxa","LF_Femur","RF_Femur","LF_Tibia","RF_Tibia",
	"LR_Coxa","RR_Coxa","LR_Femur","RR_Femur","LR_Tibia","RR_Tibia",
	"LM_Coxa","RM_Coxa","LM_Femur","RM_Femur","LM_Tibia","RM_Tibia"}
nJoints=#joints
jointsCmd={}
jointsVal={}
for i=1,nJoints do
   jointName = joints[i].."_Joint"
   handle=simGetObjectHandle(jointName)
   print ("Joint : "..jointName.." id="..handle)
   jointsId[i]=handle
   jointsCmd[#jointsCmd+1] = 0.0
   jointsVal[#jointsVal+1] = 0.0
   --simSetJointPosition (handle,0.0)
end

-- Socket Port number
portNb = 30100
serverOn = false
connexionTimeout = 0.01
cntTimeout = 0
socket=require("socket")
srv = nil
clt1 = nil

print ("end init ...")
print ("start thread ...")
-- Execute the thread function:
res=false
err=" not launched delibarately "
res,err=xpcall(threadFunction,function(err) return debug.traceback(err) end)
if not res then
   simAddStatusbarMessage('Lua runtime error: '..err)
end
