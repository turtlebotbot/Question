I'm trying to implement reinforcement learning in a carmaker with Python. But, There is a problem. The initial code was uncomfortable because the carmaker repeatedly turned on and off for each episode.
---------------Initial Learning Script------------------
  for n in range(n_episode):
        env = CMEnv(window)
        time.sleep(5)
        state, score = env.reset()
        state = state[0]
        logger_reward = []
        env.signal = 0
        action = agent.choose_action(state, n)
        env.get_signal(action)
        env.start()
        env.send_signal()
        for i, value in enumerate(target_data):
            velocity, sim_time, consump, exp_vel, _ = env.recv_data()

            print(sim_time, " /" , i/10)


            reward, next_state = env.step(tg_queue, fut_tg_list,
                                          pvel_queue, fvel_queue, consump, error)


            if i%2==0:
                done = 0
                agent.remember(state, action, reward, next_state, done)
            agent.learn()

            score += reward

            logger_reward.append(reward)

            state = next_state
            action = agent.choose_action(state, n)
            signal = env.get_signal(action)
            env.send_signal()
        env.finish()
        cnt+=1
---------------------------------------------------------------
class CMEnv():

    def __init__(self, window):
        
        IP_ADDRESS = "localhost"
        PORT = 16660
        cm_path = "C:\\IPG\\carmaker\\win64-11.0.1\\bin "
        cur_path = "D:\\cm\\CarMaker-Environment-for-RL-main\\CarMaker-Environment-for-RL-main\\carmaker "
        os.chdir(cm_path)
        os.system("CM.exe -cmdport 16660 ")
        os.chdir(cur_path)

        self.cm = CarMaker(IP_ADDRESS, PORT)
        self.cm.connect()
        self.cm.load_testrun()
        self.cm.sim_start()
        time.sleep(0.5)
        self.cm.sim_stop()
        print("[INFO] CarMaker Initialized")
        time.sleep(1)
----------------------------------------------------------------
    def sim_start(self):
      
          # msg = "StartSim\rWaitForStatus running\rWaitForTime 30\r StopSim\r"
        msg = "StartSim\r"
        self.socket.send(msg.encode())
        rsp = self.socket.recv(200)
        rsp = rsp.decode().split("\r\n\r\n")
        self.logger.info("Simulation Start : " + str(rsp))
        msg = "WaitForStatus running\r"
        self.socket.send(msg.encode())
        rsp = self.socket.recv(200)
        rsp = rsp.decode().split("\r\n\r\n")
        self.logger.info("Wait for status running : " + str(rsp))
} 
    def sim_stop(self):
       
           msg = "StopSim\r"
        self.socket.send(msg.encode())
        rsp = self.socket.recv(200)
        rsp = rsp.decode().split("\r\n\r\n")
        self.logger.info("Simulation Stop : " + str(rsp)) 
       
       
------------------------------------------------------------------
We didn't want the carmaker to end at the end of the episode. In order to repeat only the start and end of the simulation without the entire end(the closing of a window), We changed main script to the following.
----------------Modified Learning Script--------------------------
    for n in range(n_episode):
     
        env.quantity()
        param_prev, param_curr, state, score = env.reset()
        state = state[0]
        
        env.signal = 0

        env.start()
        env.send_signal()


        for i in range(100):
        
            env.cm.read()
            sim_time,ax,steer,vx,vy,yaw,kp,tx,ty,tRoad = env.recv_data()

            state = next_state
            action = agent.choose_action(state, n)
            signal = env.get_signal(action)
            env.send_signal() # 가속도는 그냥 mu받아서 gas, brake로 조절하면되니까 그대로사용.

            if isdone == 1:
                env.stop()
                print("[INFO]"+info)
                break

            if slp > 0:
                time.sleep(slp)
            else:
                pass

            if i == 80:
                env.stop()
                print("[INFO] Drive Cycle Complete")
                break
          
            
        
        
        
-------------------------------------------------------------------------------------------------------
This code runs up to two or three episodes, after that, CarMaker Stuck in Preparation Phase and the simulation is no longer started. I wonder if you can give me some advice on this situation.