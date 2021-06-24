using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class MainProject : MonoBehaviour {
    
    static List<int> nbPed_eachStep = new List<int> { 12, 15, 20, 25, 30, 35, 40, 45, 50, 60 };
    public static int cmpt_tour=0;
    static int tour_max = 3;

    // Param initialisation
    public static float ped_radius_;// = 0.3f;
    public static float time_step;// = 0.1f;
    public static float neighborDist;// = 10;
    public static int maxNeighbors;// = 10;
    public static float timeHorizon;// = 1;
    public static float timeHorizonObst;// = 1;
    public static float corridor_length_;// = 30;
    public static float corridor_width_;// = 10;
    public static bool follow_;// = true;
    public static int nb_ped;// = nbPed_eachStep[0];
    public static int time_stop;// = 30;
    public static int num_type_follow;// = 11;
    public static bool is_saved;// = false;

    //getters
    internal static float get_ped_radius_(){   return ped_radius_;}
    internal static float get_time_step(){   return time_step;}
    internal static float get_neighborDist(){   return neighborDist; }
    internal static int get_maxNeighbors() { return maxNeighbors; }
    internal static float get_timeHorizon() { return timeHorizon; }
    internal static float get_timeHorizonObst() { return timeHorizonObst; }
    internal static float get_corridor_length_() { return corridor_length_; }
    internal static float get_corridor_width_() { return corridor_width_; }
    internal static bool get_follow_() { return follow_; }
    internal static int get_nb_ped() { return nb_ped; }
    internal static int get_time_stop() { return time_stop; }
    internal static int get_num_type_follow() { return num_type_follow; }
    internal static bool get_is_saved() { return is_saved; }

    //setters
    internal static void set_is_saved(bool is_saved_newValue){   is_saved = is_saved_newValue; }
    internal static void set_nb_ped(int nb_ped_newValue) { nb_ped = nb_ped_newValue; }

    /*void Start () {
        update_param();
        SceneManager.LoadScene(0);
    }*/

    public static void update_param()
    {
        ped_radius_ = 0.3f;
        time_step = 0.1f;
        neighborDist = 10;
        maxNeighbors = 10;
        timeHorizon = 1;
        timeHorizonObst = 1;
        corridor_length_ = 30;
        corridor_width_ = 10;
        follow_ = true;
        nb_ped = nbPed_eachStep[0];
        time_stop = 30;
        num_type_follow = 11;
        is_saved = false;
    }

    // Update is called once per frame
    void Update () {
        if (get_is_saved() == true)
        {
            cmpt_tour = cmpt_tour + 1;
            //Debug.Break();
        }

        switch (cmpt_tour)
        {
            case 0:
                update_param();
                SceneManager.LoadScene(0);
                break;

            case 1:
                update_param();
                set_nb_ped(nbPed_eachStep[1]);
                SceneManager.LoadScene(0);
                break;

            case 2:
                update_param();
                set_nb_ped(nbPed_eachStep[2]);
                SceneManager.LoadScene(0);
                break;

            case 3:
                update_param();
                set_nb_ped(nbPed_eachStep[3]);
                SceneManager.LoadScene(0);
                break;

            case 4:
                update_param();
                set_nb_ped(nbPed_eachStep[4]);
                SceneManager.LoadScene(0);
                break;
        }
    }
}
