#pragma once

#include "main/scene_base/base.hpp"

#ifdef SCENE_SPHERE_COLLISION

// Structure of a particle
struct particle_structure
{
    vcl::vec3 p; // Position
    vcl::vec3 v; // Speed
    vcl::vec3 f; // Forces

    vcl::vec3 c; // Color
    float r;     // Radius
};

struct gui_scene_structure
{
    bool add_sphere = true;
    float time_interval_new_sphere = 0.5f;
    bool display_keyframe = true;
    bool display_polygon  = true;
};

// Store a vec3 (p) + time (t)
struct vec3t{
    vcl::vec3 p; // position
    float t;     // time
};

struct scene_model : scene_base
{
    /* ------ SPHERE COLLISION */

    void setup_data(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
    void frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);

    void set_gui();

    void compute_time_step(float dt);
    void create_new_particle();
    void display_particles(scene_structure& scene);
    void collision_plane(particle_structure& particle, float detect, vcl::vec3 plane, vcl::vec3 n);
    void apply_transformation(vcl::mat4 mat_trans, vcl::mat4 mat_rot);

    /* ------ INTERPOLATION POSITION */

    // Called every time the mouse is clicked
    void mouse_click(scene_structure& scene, GLFWwindow* window, int button, int action, int mods);
    // Called every time the mouse is moved
    void mouse_move(scene_structure& scene, GLFWwindow* window);

    // Data (p_i,t_i)
    vcl::buffer<vec3t> keyframes; // Given (position,time)

    vcl::mesh_drawable point_visual;                       // moving point
    vcl::mesh_drawable keyframe_visual;                    // keyframe samples
    vcl::mesh_drawable keyframe_picked;                    // showing the picked sample
    vcl::segment_drawable_immediate_mode segment_drawer;   // used to draw segments between keyframe samples
    vcl::curve_dynamic_drawable trajectory;                // Draw the trajectory of the moving point as a curve

    // Store the index of a selected sphere
    int picked_object;

    gui_scene_structure gui_scene;
    vcl::timer_interval timer_interpolation;

    /* ------ SPHERE COLLISION */

    std::vector<particle_structure> particles;
    std::vector<vcl::vec3> borders_segments;
    vcl::vec3 origin = {-1,-1,-1};
    vcl::vec3 right = {1,-1,-1};
    vcl::vec3 up = {-1,1,-1};
    vcl::vec3 forward = {-1,-1,1};
    vcl::vec3 origin_particle = {0,0,0};

    bool new_translation = false;
    // Rotation
    bool move_box_x = false;
    bool move_box_y = false;
    bool move_box_z = false;
    vcl::vec3 axis_of_rotation = {0,0,0};
    float angle_of_rotation = 0;
    vcl::vec3 new_axis_of_rotation = {0,0,0};
    float new_angle_of_rotation = 0;

    vcl::mesh_drawable sphere;      // Visual display of particles
    vcl::segments_drawable borders; // Visual display of borders
    vcl::mesh_drawable face;

    vcl::mesh_drawable test;

    vcl::timer_event timer;

    std::map<std::string,GLuint> shaders_;

};






#endif
