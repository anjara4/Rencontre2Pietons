using CenterSpace.Free;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Troschuetz.Random;
using UnityEngine.UI;
using System.Threading;
using System.IO;
using System.Linq;
using UnityEngine.SceneManagement;

namespace RVO
{
    class Samuel : Scenario
    {
        public Transform prefab;
        //public Transform human;

        public Samuel() : base() { }
        int ped_num_;
        
        bool group_ = false;
        int fluxes_ = 1;
        float corridor_angle_ = 0;
        bool loop_ = true;
        List<Color> colors = new List<Color>();
        private ManualResetEvent[] doneEvents_;

        bool is_saved;// = false;
        float ped_radius_;// = 0.3f;
        float time_step;// = 0.1f;
        float neighborDist;// = 10;
        int maxNeighbors;// = 10;
        float timeHorizon;// = 1;
        float timeHorizonObst;// = 1;
        float corridor_length_;// = 30;
        float corridor_width_;// = 10;
        bool follow_;// = true;
        static int nb_ped;// = 400;
        int time_stop;// = 300;
        static int num_type_follow;// = 11;
        List<Vector2> listPosition = new List<Vector2>();
        
        void Start()
        {
            ped_radius_ = 0.3f;
            time_step = 0.1f;
            neighborDist = 10;
            maxNeighbors = 10;
            timeHorizon = 1;
            timeHorizonObst = 1;
            corridor_length_ = 200;
            corridor_width_ = 10;
            follow_ = true;
            nb_ped = 2;
            time_stop = 30;
            num_type_follow = 11;
            is_saved = false;

            colors.Add(new Color(0.647f, 0.165f, 0.165f)); //Col_Brown
            colors.Add(new Color(0.000f, 1.000f, 1.000f)); //Col_Cyan
            colors.Add(new Color(0.000f, 0.392f, 0.000f)); //Col_DarkGreen
            colors.Add(new Color(0.282f, 0.239f, 0.545f)); //Col_DarkSlateBlue
            colors.Add(new Color(0.604f, 0.804f, 0.196f));
            colors.Add(new Color(0.804f, 0.804f, 0.196f));
            colors.Add(new Color(0.804f, 0.804f, 0.096f));
            colors.Add(new Color(0.94f, 0.804f, 0.10f));

            agents = new List<Transform>();
            Application.targetFrameRate = 60;
            Application.runInBackground = true;
            //ped_num_ = (int)(1.9f * corridor_width_ * (corridor_length_ + 10));
            ped_num_ = nb_ped;
            sim_.setTimeStep(time_step);
            transform.localScale = new Vector3(corridor_length_ / 10, 1, corridor_width_ / 10);
            transform.position = new Vector3(corridor_length_ / 2, 0, corridor_width_ / 2);
            //my_corridor
            transform.GetComponent<MeshRenderer>().material.color = colors[0];
            setupScenario();
            //follow_but_.isOn = follow_;


            sim_.initialize_virtual_and_agents();
            sim_.processObstacles();

            sim_.kdTree_.buildAgentTree(true);
        }

        
        // Update is called once per frame
        void Update()
        {
            if (!reachedGoal())
            {
                setAgentsProperties();
                setPreferredVelocities();
                sim_.initialize_virtual_and_agents();
                
                for (int i = 0; i < getNumAgents(); i++)
                {
                    using (TextWriter tw = new StreamWriter("test.csv", true))
                        tw.WriteLine( i +"\t" + sim_.globalTime_ + "\t" + sim_.getAgentPosition(i).x_ + "\t" + sim_.getAgentPosition(i).y_ + "\t" + sim_.getAgentVelocity(i).x_ + "\t" + sim_.getAgentVelocity(i).y_);

                    Vector2 agent_position = sim_.getAgentPosition(i);
                    if (agent_position.x_ < 5)
                    {
                        Vector2 p1 = agent_position + new Vector2(corridor_length_, 0);
                        sim_.addVirtualAgent(0, p1);
                    }

                }

                doStep(true);

                Vector3 pos3 = Camera.main.transform.position;
                //Camera.main.transform.position = new Vector3(pos3.x, camera_height_.value, pos3.z);
                Camera.main.transform.position = new Vector3(15, 40, 1);
                //Camera.main.transform.rotation = new Quaternion(90, 0, 0, 0);
                //Camera.main.transform.Rotate(90, 0, 0);
                Camera.main.transform.rotation = Quaternion.LookRotation(new Vector3(0, -90, 0));

                int totot = getNumAgents();
                for (int i = 0; i < getNumAgents(); ++i)
                {
                    Vector2 position = sim_.getAgentPosition(i);
                    agents[i].transform.position = new Vector3(position.x(), 0f, position.y());
                    RVO.Vector2 vector_cam = sim_.getAgentVelocity(i);
                    agents[i].rotation = Quaternion.LookRotation(new Vector3(vector_cam.x_, 0, vector_cam.y_));

                }
            }
            else
            {
                for (int i = 0; i < getNumAgents(); ++i)
                {
                    agents[i].transform.GetComponent<Rigidbody>().isKinematic = true;
                }
            }

            
        }


        void setCorridor()
        {// Add (polygonal) obstacle(s), specifying vertices in counterclockwise order.
         // Add corridor right side

            IList<Vector2> right_side = new List<Vector2> {
            Vector2.rotation(new Vector2(corridor_length_ + 100, 0.0f), corridor_angle_),
            Vector2.rotation(new Vector2(-100.0f, 0.0f), corridor_angle_),
            Vector2.rotation(new Vector2(-100.0f, -50.0f), corridor_angle_),
            Vector2.rotation(new Vector2(corridor_length_ + 100, -50.0f), corridor_angle_)};
            sim_.addObstacle(right_side);
            //Add cooridor left side
            IList<Vector2> left_side = new List<Vector2> {
            Vector2.rotation(new Vector2(-100.0f, corridor_width_), corridor_angle_),
            Vector2.rotation(new Vector2(corridor_length_ + 100, corridor_width_), corridor_angle_),
            Vector2.rotation(new Vector2(corridor_length_ + 100, corridor_width_ + 50.0f), corridor_angle_),
            Vector2.rotation(new Vector2(-100.0f, corridor_width_ + 50.0f), corridor_angle_) };
            sim_.addObstacle(left_side);


            IList<Vector2> begin_side = new List<Vector2> {
             Vector2.rotation(new Vector2(-0, -50), corridor_angle_),
             Vector2.rotation(new Vector2(-0 , corridor_width_+50), corridor_angle_),
             Vector2.rotation(new Vector2(-10, corridor_width_ + 50.0f), corridor_angle_),
             Vector2.rotation(new Vector2(-10, 0), corridor_angle_) };
            sim_.addObstacle(begin_side);
            // Process obstacles so that they are accounted for in the simulation.
            sim_.processObstacles();
        }

        void placeAgents()
        {
            File.Create("test.csv").Dispose();
            using (TextWriter tw = new StreamWriter("test.csv"))
                tw.WriteLine("numAgent" + "\t" + "timeSimu" + "\t" + "pX" + "\t" + "pY" + "\t" + "vX" + "\t" + "vY");

            Vector2 position = new Vector2(10, 2);
            //position = Vector2.rotation(position, corridor_angle_);

            Vector2 position2 = new Vector2(40, 2);
            //position2 = Vector2.rotation(position2, corridor_angle_);

            //listPosition.Add(position);
            //listPosition.Add(position2);

            sim_.addRVOAgent(position, follow_, group_, 10, 10, 1, 1, ped_radius_, 0.1f, new RVO.Vector2(0, 0), 0.5f);
            sim_.addRVOAgent(position2, follow_, group_, 10, 10, 1, 1, ped_radius_, 0.1f, new RVO.Vector2(0, 0), 0.5f);



            addAgent(prefab, new Vector3(position.x(), 0, position.y()), sim_.getDefaultRadius());
            addAgent(prefab, new Vector3(position2.x(), 0, position2.y()), sim_.getDefaultRadius());

            //step_stop.Add(0);

            // Set agent's goal
            Vector2 corridor_end = new Vector2(corridor_length_, position.y());
            corridor_end = Vector2.rotation(corridor_end, corridor_angle_);
            sim_.setAgentGoal(0, corridor_end);

            Vector2 corridor_end2 = new Vector2(0, position2.y());
            corridor_end2 = Vector2.rotation(corridor_end2, corridor_angle_);
            sim_.setAgentGoal(1, corridor_end2);
        }

        void setAgentsProperties()
        {
            for (int i = 0; i < sim_.getNumAgents(); ++i)
            {  // Set Agent Goal
                Vector2 pos = sim_.getAgentPosition(i);
                Vector2 goal = sim_.getAgentGoal(i);
                // Position in the corridor referential
                Vector2 local_pos = Vector2.rotation(pos, -corridor_angle_);
                Vector2 local_goal = Vector2.rotation(goal, -corridor_angle_);
                // Set agent goal
                Vector2 new_goal = new Vector2(local_goal.x(), local_pos.y());
                // Back to world's referential
                new_goal = Vector2.rotation(new_goal, corridor_angle_);
                // Set goal
                sim_.setAgentGoal(i, new_goal);
                
                if (local_pos.x() >= corridor_length_ -5 && local_goal.x() > corridor_length_ - 5)
                {
                    // Put at the start of the corridor
                    Vector2 new_pos = listPosition[i];
                    // Back to world's referential
                    new_pos = Vector2.rotation(new_pos, corridor_angle_);
                    // Add agent
                    sim_.setAgentPosition(i, new_pos);

                }
            }
        }

        public override void setupScenario()
        {
            setCorridor();
            placeAgents();
        }
    }
}
