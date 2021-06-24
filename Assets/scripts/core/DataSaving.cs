using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

namespace RVO
{
    class DataSaving
    {


        DataSaving()
        { }


        

        // Save the data of each agent (position, velocity, acceleration, distance with the closer, local density and agent's leader
        internal static void saveData(float time, float My_num, float pos_x, float Num_voisin, float X_Voisin, float distance)
        {
            //System.IO.Directory.CreateDirectory(path_keep_data);
            if (!File.Exists("Data/Ped_VS_PedImmobile/" + "TEST" + "_01.csv"))
                using (TextWriter tw = new StreamWriter("Data/Ped_VS_PedImmobile/" + "TEST" + "_01.csv"))
                    tw.WriteLine("Time" + "\t" + "My_num" + "\t" + "X" + "\t" + "Num_voisin" + "\t" + "X_Voisin" + "\t" + "Distance");

            using (TextWriter tw = new StreamWriter("Data/Ped_VS_PedImmobile/" + "TEST" + "_01.csv", true))  // 
                tw.WriteLine(time + "\t" + My_num + "\t" + pos_x + "\t" + Num_voisin + "\t" + X_Voisin + "\t" + distance);
        }

        internal static void saveDataFollowerActivation(float time, int My_num, float pos_x, float pos_y, double speed)
        {
            string path_keep_data = "Data/Ped_Analyse_Follow/";
            //System.IO.Directory.CreateDirectory(path_keep_data);
            if (!File.Exists(path_keep_data + "Activation" + "_01.csv"))
                using (TextWriter tw = new StreamWriter(path_keep_data + "Activation" + "_01.csv"))
                    tw.WriteLine("Time" + "\t" + "My_num" + "\t" + "X" + "\t" + "Y" + "\t" + "V" + "\t" + "Activation");

            using (TextWriter tw = new StreamWriter(path_keep_data + "Activation" + "_01.csv", true))  // 
                tw.WriteLine(time + "\t" + My_num + "\t" + pos_x + "\t" + pos_y + "\t" + speed + "\t" + "TRUE");
        }

        internal static void saveDataFollower(float time, int My_num, float pos_x, float pos_y, double speed)
        {
            string path_keep_data = "Data/Ped_Analyse_Follow/";
            //System.IO.Directory.CreateDirectory(path_keep_data);
            if (!File.Exists(path_keep_data + "General" + "_01.csv"))
                using (TextWriter tw = new StreamWriter(path_keep_data + "General" + "_01.csv"))
                    tw.WriteLine("Time" + "\t" + "My_num" + "\t" + "X" + "\t" + "Y" + "\t" + "V" + "\t" + "Activation");

            using (TextWriter tw = new StreamWriter(path_keep_data + "General" + "_01.csv", true))  // 
                tw.WriteLine(time + "\t" + My_num + "\t" + pos_x + "\t" + pos_y + "\t" + speed + "\t" + "");
        }

        // Save the agent's data when it attempts the end of the corridor
        //Not used
        static void saveAgentData(RVOSimulator sim, int agentNo, bool follow)
        {
            string name = "Data/n";
            string num = sim.getNumAgents().ToString();
            name += num;
            if (follow)
                name += "f";
            name += "_end_data.csv";


            System.IO.File.WriteAllText(@name, agentNo.ToString() + '\t' + sim.getAgentCptDist(agentNo).ToString() + '\t' + sim.getAgentCptTime(agentNo));


        }


        public static void saveSimulatorData(RVOSimulator sim, bool follow)
        {
            string name = "Agents_moy_data.csv";


            if (!File.Exists(name))
            {
                string tmp = " ";
                foreach (Agent a in sim.agents_)
                {
                    tmp += a.id_ + " : vitesse_moy \t";
                }
                File.Create(name).Dispose();
                using (TextWriter tw = new StreamWriter(name))
                {
                    tw.WriteLine(tmp);
                }
            }



            using (TextWriter tw = new StreamWriter(name, true))
            {
                string tmp = " ";
                foreach (Agent a in sim.agents_)
                {
                    tmp += " " + Vector2.abs(a.velocity_) + "\t";
                }
                tw.WriteLine(tmp);
            }


        }

    }

}

