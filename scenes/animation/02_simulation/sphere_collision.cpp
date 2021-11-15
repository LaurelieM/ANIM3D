
#include "sphere_collision.hpp"
#include "vcl/math/mat/mat4/mat4.hpp"
#include <random>

#ifdef SCENE_SPHERE_COLLISION

using namespace vcl;

/* ------ INTERPOLATION POSITION FUNCTION */

/** Function returning the index i such that t \in [v[i].t,v[i+1].t] */
static size_t index_at_value(float t, const vcl::buffer<vec3t> &v);

static vec3 linear_interpolation(float t, float t1, float t2, const vec3& p1, const vec3& p2);
static vec3 cardinal_spline_interpolation(float t, float t0, float t1, float t2, float t3, const vec3& p0, const vec3& p1, const vec3& p2, const vec3& p3);

/* ------ ALL */

void scene_model::apply_transformation(mat4 mat_trans, mat4 mat_rot) {
    /* Appliquer translation et rotation aux points */
    vec4 new_origin = mat_trans *  mat_rot * vec4{origin[0], origin[1], origin[2], 1};
    vec4 new_up = mat_trans *  mat_rot * vec4{up[0], up[1], up[2], 1};
    vec4 new_right = mat_trans *  mat_rot * vec4{right[0], right[1], right[2], 1};
    vec4 new_forward = mat_trans *  mat_rot * vec4{forward[0], forward[1], forward[2], 1};

    /* Calcul des nouveaux points en omettant la 4e dim */
    origin = vec3{new_origin[0], new_origin[1], new_origin[2]};
    up = vec3{new_up[0], new_up[1], new_up[2]};
    right = vec3{new_right[0], new_right[1], new_right[2]};
    forward = vec3{new_forward[0], new_forward[1], new_forward[2]};
}

void scene_model::frame_draw(std::map<std::string,GLuint>& , scene_structure& scene, gui_structure& )
{
    /* ------ INTERPOLATION POSITION */
    timer_interpolation.update();
    const float t = timer_interpolation.t;

    if( t<timer_interpolation.t_min+0.1f ) // clear trajectory when the timer restart
        trajectory.clear();

    set_gui();

    // ********************************************* //
    // Compute interpolated position at time t
    // ********************************************* //
    const int idx = index_at_value(t, keyframes);

    // Assume a closed curve trajectory
    const size_t N = keyframes.size();

    // Preparation of data for the linear interpolation
    // Parameters used to compute the linear interpolation
    const float t1 = keyframes[idx  ].t; // = t_i
    const vec3& p1 = keyframes[idx  ].p; // = p_i

    float t0 = t1;
    float t2 = t1;
    float t3 = t1;
    vec3 p0 = p1;
    vec3 p2 = p1;
    vec3 p3 = p1;

    if (idx == 0) {
        t2 = keyframes[idx+1].t; // = t_{i+1}
        t3 = keyframes[idx+2].t; // = t_{i+2}
        p2 = keyframes[idx+1].p; // = p_{i+1}
        p3 = keyframes[idx+2].p; // = p_{i+2}
    } else if (idx == N-2) {
        t0 = keyframes[idx-1].t; // = t_{i-1}
        t2 = keyframes[idx+1].t; // = t_{i+1}
        t3 = keyframes[idx+1].t; // = t_{i+1}
        p0 = keyframes[idx-1].p; // = p_{i-1}
        p2 = keyframes[idx+1].p; // = p_{i+1}
        p3 = keyframes[idx+1].p; // = p_{i+1}
    } else if (idx == N-1) {
        t0 = keyframes[idx-1].t; // = t_{i-1}
        p0 = keyframes[idx-1].p; // = p_{i-1}
    } else {
        t0 = keyframes[idx-1].t; // = t_{i-1}
        t2 = keyframes[idx+1].t; // = t_{i+1}
        t3 = keyframes[idx+2].t; // = t_{i+2}
        p0 = keyframes[idx-1].p; // = p_{i-1}
        p2 = keyframes[idx+1].p; // = p_{i+1}
        p3 = keyframes[idx+2].p; // = p_{i+2}
    }    

    // Compute the linear interpolation here
    //const vec3 p = linear_interpolation(t,t1,t2,p1,p2);

    // Create and call a function cardinal_spline_interpolation(...) instead
    // ...
    const vec3 p = cardinal_spline_interpolation(t,t0,t1,t2,t3,p0,p1,p2,p3);


    // Store current trajectory of point p
    trajectory.add_point(p);


    // Draw current position
    point_visual.uniform.transform.translation = p;
    draw(point_visual, scene.camera);

    // Draw moving point trajectory
    trajectory.draw(shaders_["curve"], scene.camera);


    // Draw sphere at each keyframe position
    if(gui_scene.display_keyframe) {
        for(size_t k=0; k<N; ++k)
        {
            const vec3& p_keyframe = keyframes[k].p;
            keyframe_visual.uniform.transform.translation = p_keyframe;
            draw(keyframe_visual, scene.camera);
        }
    }

    // Draw selected sphere in red
    if( picked_object!=-1 )
    {
        const vec3& p_keyframe = keyframes[picked_object].p;
        keyframe_picked.uniform.transform.translation = p_keyframe;
        draw(keyframe_picked, scene.camera);
    }


    // Draw segments between each keyframe
    if(gui_scene.display_polygon) {
        for(size_t k=0; k<keyframes.size()-1; ++k)
        {
            const vec3& pa = keyframes[k].p;
            const vec3& pb = keyframes[k+1].p;

            segment_drawer.uniform_parameter.p1 = pa;
            segment_drawer.uniform_parameter.p2 = pb;
            segment_drawer.draw(shaders_["segment_im"], scene.camera);
        }
    }

    /* ------ SPHERE COLLISION */

/*
    origin = p;
    right = origin + vec3{1,0,0};
    up = origin + vec3{0,1,0};
    forward = origin + vec3{0,0,1};
    origin_particle = origin + vec3{0.5,0.5,0.5};*/

    float dt = 0.02f * timer.scale;
    timer.update();

    create_new_particle();
    compute_time_step(dt);

    display_particles(scene);

    /* Translation */
    mat4 mat_trans = mat4::identity();
    if (new_translation) {
        auto center_box = origin + vec3{(right[0]-origin[0])/2, (up[1]-origin[1])/2, (forward[2]-origin[2])/2};
        vec3 translation = p - center_box; // on fait bouger le cube sur la courbe
        std::cout << p << center_box << std::endl;
        mat_trans = mat4::from_translation(translation);
        test.uniform.transform.translation = translation;
        origin_particle = p;
        apply_transformation(mat_trans, mat4::identity());
    }

    /* Rotation */
    mat4 mat_rot = mat4::identity();
    if (angle_of_rotation != new_angle_of_rotation) {
        vec3 translation = -p; // on remet le cube au centre
        mat4 mat_trans = mat4::from_translation(translation);
        test.uniform.transform.translation = translation;
        apply_transformation(mat_trans, mat4::identity());
        auto center_box = origin + vec3{(right[0]-origin[0])/2, (up[1]-origin[1])/2, (forward[2]-origin[2])/2};

        mat3 rotation = rotation_from_axis_angle_mat3(new_axis_of_rotation, new_angle_of_rotation-angle_of_rotation);
        std::cout << new_axis_of_rotation << " - " << new_angle_of_rotation-angle_of_rotation << std::endl;
        mat_rot = mat4::from_mat3(rotation);
        //apply_transformation(mat4::identity(), mat_rot);
        axis_of_rotation = new_axis_of_rotation;
        angle_of_rotation = new_angle_of_rotation;
        
        translation = p;
        mat_trans = mat4::from_translation(translation);
        test.uniform.transform.translation = translation;
        apply_transformation(mat_trans, mat_rot);

        center_box = origin + vec3{(right[0]-origin[0])/2, (up[1]-origin[1])/2, (forward[2]-origin[2])/2};
    }

    auto test_shape = mesh_primitive_parallelepiped(origin, up - origin, right - origin, forward - origin);

    test = mesh_drawable(test_shape); // Note that a mesh_drawable can be directly constructed from a mesh
    test.uniform.color = {0.6f, 0.3f, 0};
    test.shader = shaders_["curve"];

    borders_segments =  {};
    for (size_t i = 0; i < test_shape.position.size(); ++i)
    {
        borders_segments.push_back(test_shape.position[i]);
    }

    borders = segments_gpu(borders_segments);
    borders.uniform.color = {0,0,0};
    borders.shader = shaders_["curve"];

    draw(borders, scene.camera);
    //draw(test, scene.camera);
}


void scene_model::collision_plane(particle_structure& particle, float detect, vec3 plane, vec3 n)
{
    auto vperp= dot(particle.v, n) * n;
    auto vpara= particle.v - vperp;
    particle.v = vpara - vperp;
    auto d = particle.r - detect;
    particle.p = particle.p + d * n;
}

void scene_model::compute_time_step(float dt)
{
    // Set forces
    const size_t N = particles.size();
    for(size_t k=0; k<N; ++k)
        particles[k].f = vec3(0,-9.81f,0);


    // Integrate position and speed of particles through time
    for(size_t k=0; k<N; ++k) {
        particle_structure& particle = particles[k];
        vec3& v = particle.v;
        vec3& p = particle.p;
        vec3 const& f = particle.f;

        v = (1-0.9f*dt) * v + dt * f; // gravity + friction force
        p = p + dt * v;
    }

   for(size_t k=0; k<N; ++k)
   {
       /* Collisions with cube */

       particle_structure& particle = particles[k];
       //centre du cube:
       vec3 center_box = origin_particle;

       //face droite du cube
       vec3 A = borders_segments[11];
       vec3 B = borders_segments[3];
       vec3 C = borders_segments[1];
       vec3 plane = (A + C)/2;
       vec3 n = center_box - plane;
       n = n / norm(n);

       auto detect = dot((particle.p - plane), n);
       if (fabs(detect) <= particle.r)
       {
           //std::cout << "pass DROIT plane: "<<plane<< " n: "<< n<< " centerbox: "<< center_box << '\n'<<std::endl;
           collision_plane(particle, detect, plane, n);
       }

       //face gauche du cube
       A = borders_segments[0];
       B = borders_segments[5];
       C = borders_segments[13];
       plane = (A + C)/2;
       n = center_box - plane;
       n = n / norm(n);

       detect = dot((particle.p - plane), n);
       if (fabs(detect) <= particle.r)
       {
           //std::cout << "pass gauche plane: "<<plane<< " n: "<< n<< " centerbox: "<< center_box <<std::endl;
           //std::cout << "origin: " << origin <<" right " << right << " up " << up << " forw " << forward<<'\n'<<std::endl;
           collision_plane(particle, detect, plane, n);
       }

       //Face du dessus du cube
       A = borders_segments[5];
       B = borders_segments[3];
       C = borders_segments[11];
       plane = (A + C)/2;
       n = center_box - plane;
       n = n / norm(n);

       detect = dot((particle.p - plane), n);
       if (fabs(detect) <= particle.r)
       {
           //std::cout << "pass dessus plane: "<<plane<< " n: "<< n<< " centerbox: "<< center_box << '\n'<<std::endl;
           collision_plane(particle, detect, plane, n);
       }

       //face basse du cube
       A = borders_segments[0];
       B = borders_segments[1];
       C = borders_segments[9];
       plane = (A + C)/2;
       n = center_box - plane;
       n = n / norm(n);

       detect = dot((particle.p - plane), n);
       if (fabs(detect) <= particle.r)
       {
           //std::cout << "pass basse plane: "<<plane<< " n: "<< n<< " centerbox: "<< center_box << '\n'<<std::endl;
           collision_plane(particle, detect, plane, n);
       }

       //Face avant du cube
       A = borders_segments[13];
       B = borders_segments[11];
       C = borders_segments[9];
       plane = (A + C)/2;
       n = center_box - plane;
       n = n / norm(n);

       detect = dot((particle.p - plane), n);
       if (fabs(detect) <= particle.r)
       {
           //std::cout << "pass avant plane: "<<plane<< " n: "<< n<< " centerbox: "<< center_box << '\n'<<std::endl;
           collision_plane(particle, detect, plane, n);
       }

       //face arrière du cube
       A = borders_segments[0];
       B = borders_segments[5];
       C = borders_segments[3];
       plane = (A + C)/2;
       n = center_box - plane;
       n = n / norm(n);

       detect = dot((particle.p - plane), n);
       if (fabs(detect) <= particle.r)
       {
           //std::cout << "pass arrière plane: "<<plane<< " n: "<< n<< " centerbox: "<< center_box << '\n'<<std::endl;
           collision_plane(particle, detect, plane, n);
       }

       /* Collision with spheres */
       for(size_t k2=0; k2<N && k!=k2; ++k2)
       {
           particle_structure& particle2 = particles[k2];
           auto detect = norm((particle.p - particle2.p));
           if (detect <= particle.r + particle2.r +0.02f)
           {
               auto m1 = 1;
               auto m2 = 1;
               auto u = (particle.p - particle2.p) / detect;
               //auto j = 2 * ((m1 * m2)/(m1 + m2)) * dot((particle2.v - particle.v), u);

               auto tmp = particle.v;
               particle.v = particle.v + dot((particle2.v - particle.v), u) * u;
               particle2.v = particle2.v - dot((particle2.v - tmp), u) * u;

               auto d = particle.r + particle2.r - detect;
               particle.p = particle.p + (d/2) * u;
           }
       }
   }
}


void scene_model::create_new_particle()
{
    // Emission of new particle if needed
    timer.periodic_event_time_step = gui_scene.time_interval_new_sphere;
    const bool is_new_particle = timer.event;
    static const std::vector<vec3> color_lut = {{1,0,0},{0,1,0},{0,0,1},{1,1,0},{1,0,1},{0,1,1}};

    if( is_new_particle && gui_scene.add_sphere)
    {
        particle_structure new_particle;

        new_particle.r = 0.08f;
        new_particle.c = color_lut[int(rand_interval()*color_lut.size())];

        // Initial position
        new_particle.p = origin_particle;

        // Initial speed
        const float theta = rand_interval(0, 2*3.14f);
        new_particle.v = vec3( 2*std::cos(theta), 5.0f, 2*std::sin(theta));

        particles.push_back(new_particle);
    }
}
void scene_model::display_particles(scene_structure& scene)
{
    const size_t N = particles.size();
    for(size_t k=0; k<N; ++k)
    {
        const particle_structure& part = particles[k];

        sphere.uniform.transform.translation = part.p;
        sphere.uniform.transform.scaling = part.r;
        sphere.uniform.color = part.c;
        draw(sphere, scene.camera);
    }
}




void scene_model::setup_data(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& )
{
    /* ------ INTERPOLATION POSITION */
    
    // Initial Keyframe data vector of (position, time)
    keyframes = { { {-2,2,0}   , 0.0f  },
                  { {0,2,0}    , 1.0f  },
                  { {2,2,0}    , 2.0f  },
                  { {2,3,0}    , 2.5f  },
                  { {3,3,0}    , 3.0f  },
                  { {3,3,2}    , 3.5f  },
                  { {3,0,2.5}  , 3.75f  },
                  { {2.5,-2,2} , 4.5f  },
                  { {2.5,-2,0} , 5.0f  },
                  { {2,-2,0}   , 6.0f  },
                  { {0,-1.5,0} , 7.0f },
                  { {-2,-1.5,0}, 8.0f },
                  { {-2,2,0}   , 9.0f  }
                };

    // Set timer bounds
    // You should adapt these extremal values to the type of interpolation
    timer_interpolation.t_min = keyframes[0].t;                   // first time of the keyframe
    timer_interpolation.t_max = keyframes[keyframes.size()-1].t;  // last time of the keyframe
    timer_interpolation.t = timer_interpolation.t_min;
    timer_interpolation.scale = 0.0f;

    // Prepare the visual elements
    point_visual = mesh_primitive_sphere();
    point_visual.shader = shaders["mesh"];
    point_visual.uniform.color   = {0,0,1};
    point_visual.uniform.transform.scaling = 0.04f;

    keyframe_visual = mesh_primitive_sphere();
    keyframe_visual.shader = shaders["mesh"];
    keyframe_visual.uniform.color = {1,1,1};
    keyframe_visual.uniform.transform.scaling = 0.05f;

    keyframe_picked = mesh_primitive_sphere();
    keyframe_picked.shader = shaders["mesh"];
    keyframe_picked.uniform.color = {1,0,0};
    keyframe_picked.uniform.transform.scaling = 0.055f;

    segment_drawer.init();

    trajectory = curve_dynamic_drawable(100); // number of steps stored in the trajectory
    trajectory.uniform.color = {0,0,1};

    picked_object=-1;
    scene.camera.scale = 15.0f;

    /* ------ SPHERE COLLISION */

    vec3 translation = keyframes[0].p; // on fait bouger le cube sur la courbe
    mat4 mat_trans = mat4::from_translation(translation);
    test.uniform.transform.translation = translation;
    origin_particle = keyframes[0].p;
    apply_transformation(mat_trans, mat4::identity());

    shaders_ = shaders;
    sphere = mesh_drawable( mesh_primitive_sphere(1.0f));
    sphere.shader = shaders["mesh"];

    auto test_shape = mesh_primitive_parallelepiped(origin, up - origin,  right - origin, forward - origin);

    test = mesh_drawable(test_shape); // Note that a mesh_drawable can be directly constructed from a mesh
    test.shader = shaders["curve"];
    borders_segments =  {};
    for (size_t i = 0; i < test_shape.position.size(); ++i)
    {
        borders_segments.push_back(test_shape.position[i]);
    }

    borders = segments_gpu(borders_segments);
    borders.uniform.color = {0,0,0};
    borders.shader = shaders["curve"];

    gui_scene.add_sphere = false;
}



void scene_model::set_gui()
{
    // Can set the speed of the animation
    ImGui::Text("-- Sphere: ");
    ImGui::SliderFloat("Time scale", &timer.scale, 0.05f, 2.0f, "%.2f s");
    ImGui::SliderFloat("Interval create sphere", &gui_scene.time_interval_new_sphere, 0.05f, 2.0f, "%.2f s");
    ImGui::Checkbox("Add sphere", &gui_scene.add_sphere);
    
    ImGui::Text("-- Courbe: ");
    ImGui::SliderFloat("Time interpol", &timer_interpolation.t, timer_interpolation.t_min, timer_interpolation.t_max);
    ImGui::SliderFloat("Time interpol scale", &timer_interpolation.scale, 0.0f, 1.5f);
    if (timer_interpolation.scale == 0) new_translation = false;
    else new_translation = true;

    ImGui::Text("Display: "); ImGui::SameLine();
    ImGui::Checkbox("keyframe", &gui_scene.display_keyframe); ImGui::SameLine();
    ImGui::Checkbox("polygon", &gui_scene.display_polygon);

    if( ImGui::Button("Print Keyframe") )
    {
        std::cout<<"keyframe_position={";
        for(size_t k=0; k<keyframes.size(); ++k)
        {
            const vec3& p = keyframes[k].p;
            std::cout<< "{"<<p.x<<"f,"<<p.y<<"f,"<<p.z<<"f}";
            if(k<keyframes.size()-1)
                std::cout<<", ";
        }
        std::cout<<"}"<<std::endl;
    }
    
    // Rotation de la box
    ImGui::Text("If you want to rotate the box. Please put the Time interpolate to 0."); 
    ImGui::Text("Rotate Box: "); ImGui::SameLine();
    ImGui::Checkbox("x", &move_box_x); ImGui::SameLine();
    ImGui::Checkbox("y", &move_box_y); ImGui::SameLine();
    ImGui::Checkbox("z", &move_box_z); ImGui::SameLine();
    ImGui::SliderFloat("Angle", &new_angle_of_rotation, -3.14, 3.14);
    if (move_box_x)
    {
        new_axis_of_rotation[0] = 1;
    }
    else new_axis_of_rotation[0] = 0;
    if (move_box_y)
    {
        new_axis_of_rotation[1] = 1;
    }
    else new_axis_of_rotation[1] = 0;
    if (move_box_z)
    {
        new_axis_of_rotation[2] = 1;
    }
    else new_axis_of_rotation[2] = 0;
    if (!move_box_x && !move_box_y && !move_box_z) new_angle_of_rotation = 0; 

    ImGui::Text("-----");
    bool stop_anim  = ImGui::Button("Stop"); ImGui::SameLine();
    bool start_anim = ImGui::Button("Start");
    if(stop_anim)
    {
        timer.stop();
        timer_interpolation.stop();
    }
    if(start_anim)
    {
        timer.start();
        timer_interpolation.start();
    }
}


/* ------ INTERPOLATION POSITION */

void scene_model::mouse_click(scene_structure& scene, GLFWwindow* window, int , int , int )
{
    // Mouse click is used to select a position of the control polygon
    // ******************************************************************** //

    // Cursor coordinates
    const vec2 cursor = glfw_cursor_coordinates_window(window);

    // Check that the mouse is clicked (drag and drop)
    const bool mouse_click_left  = glfw_mouse_pressed_left(window);
    const bool key_shift = glfw_key_shift_pressed(window);

    // Check if shift key is pressed
    if(mouse_click_left && key_shift)
    {
        // Create the 3D ray passing by the selected point on the screen
        const ray r = picking_ray(scene.camera, cursor);

        // Check if this ray intersects a position (represented by a sphere)
        //  Loop over all positions and get the intersected position (the closest one in case of multiple intersection)
        const size_t N = keyframes.size();
        picked_object = -1;
        float distance_min = 0.0f;
        for(size_t k=0; k<N; ++k)
        {
            const vec3 c = keyframes[k].p;
            const picking_info info = ray_intersect_sphere(r, c, 0.1f);

            if( info.picking_valid ) // the ray intersects a sphere
            {
                const float distance = norm(info.intersection-r.p); // get the closest intersection
                if( picked_object==-1 || distance<distance_min ){
                    distance_min = distance;
                    picked_object = k;
                }
            }
        }
    }

}

void scene_model::mouse_move(scene_structure& scene, GLFWwindow* window)
{

    const bool mouse_click_left  = glfw_mouse_pressed_left(window);
    const bool key_shift = glfw_key_shift_pressed(window);
    if(mouse_click_left && key_shift && picked_object!=-1)
    {
        // Translate the selected object to the new pointed mouse position within the camera plane
        // ************************************************************************************** //

        // Get vector orthogonal to camera orientation
        const mat4 M = scene.camera.camera_matrix();
        const vec3 n = {M(0,2),M(1,2),M(2,2)};

        // Compute intersection between current ray and the plane orthogonal to the view direction and passing by the selected object
        const vec2 cursor = glfw_cursor_coordinates_window(window);
        const ray r = picking_ray(scene.camera, cursor);
        vec3& p0 = keyframes[picked_object].p;
        const picking_info info = ray_intersect_plane(r,n,p0);

        // translate the position
        p0 = info.intersection;

    }
}


static size_t index_at_value(float t, vcl::buffer<vec3t> const& v)
{
    const size_t N = v.size();
    assert(v.size()>=2);
    assert(t>=v[0].t);
    assert(t<v[N-1].t);

    size_t k=0;
    while( v[k+1].t<t )
        ++k;
    return k;
}


static vec3 linear_interpolation(float t, float t1, float t2, const vec3& p1, const vec3& p2)
{
    const float alpha = (t-t1)/(t2-t1);
    const vec3 p = (1-alpha)*p1 + alpha*p2;

    return p;
}

static vec3 cardinal_spline_interpolation(float t, float t0, float t1, float t2, float t3, const vec3& p0, const vec3& p1, const vec3& p2, const vec3& p3)
{
    float nu = 1;
    auto d1 = nu * (p2-p0)/(t2-t0);
    auto d2 = nu * (p3-p1)/(t3-t1);
    auto s = (t-t1)/(t2-t1);
    const vec3 p = (2*s*s*s -3*s*s +1)*p1 + (s*s*s -2*s*s + s)*d1 + (-2*s*s*s + 3*s*s)*p2 + (s*s*s -s*s)*d2;
    return p;
}


#endif
