#include "planar_quadrotor_visualizer.h"

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor* quadrotor_ptr) : quadrotor_ptr(quadrotor_ptr) {}

/**
 * TODO: Improve visualizetion
 * 1. Transform coordinates from quadrotor frame to image frame so the circle is in the middle of the screen
 * 2. Use more shapes to represent quadrotor (e.x. try replicate http://underactuated.mit.edu/acrobot.html#section3 or do something prettier)
 * 3. Animate proppelers
 */
void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer>& gRenderer) {
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    float q_x, q_y, q_theta, q_y2;

    /* x, y, theta coordinates */
    q_x = state[0];
    q_y = state[1];
    q_theta = state[2];
    int qheight = 20;
    int qwidth = 200;
    int qcolor = 0xFFFF00FF;
    int qcolor2 = 0xF000F0FF;
    int kijHeight = 40;
    int kijWidth = 7;
    int kijkolor = 0xF0F0F0F0;
    float o = 0.85;
    // dla tablic [0] - wspolrzedna x, [1] wspolrzedna y
    float qlewo[2] = { q_x - qwidth / 2 * cos(q_theta),q_y - qwidth / 2 * sin(q_theta) };
    float qprawo[2] = { q_x + qwidth / 2 * cos(q_theta),q_y + qwidth / 2 * sin(q_theta) };

    float kijDistanceX = qwidth * o * cos(q_theta);
    float kijDistanceY = qwidth * o * sin(q_theta);

    float kijDlugoscX = kijHeight * sin(q_theta);
    float kijDlugoscY = kijHeight * cos(q_theta);

    float lewykij1[2] = { qprawo[0] - kijDistanceX, qprawo[1] - kijDistanceY };
    float lewykij2[2] = { qprawo[0] - kijDistanceX + kijDlugoscX, qprawo[1] - kijDistanceY - kijDlugoscY };
    float prawykij1[2] = { qlewo[0] + kijDistanceX, qlewo[1] + kijDistanceY };
    float prawykij2[2] = { qlewo[0] + kijDistanceX + kijDlugoscX, qlewo[1] + kijDistanceY - kijDlugoscY };

    static float wirnik_angle = 0;
    wirnik_angle += 0.01; 

    float wirnik_length = 50; 
    float wirnik_theta = wirnik_angle;

    float wirnik_center[2] = { lewykij2[0], lewykij2[1] };

    float wirnik_left_end[2] = { wirnik_center[0] - wirnik_length / 2 * cos(wirnik_theta), wirnik_center[1] - wirnik_length / 2 * sin(wirnik_theta) };
    float wirnik_right_end[2] = { wirnik_center[0] + wirnik_length / 2 * cos(wirnik_theta), wirnik_center[1] + wirnik_length / 2 * sin(wirnik_theta) };


    float wirnik_center2[2] = { prawykij2[0], prawykij2[1] };

    float wirnik_left_end2[2] = { wirnik_center2[0] - wirnik_length / 2 * cos(wirnik_theta), wirnik_center2[1] - wirnik_length / 2 * sin(wirnik_theta) };
    float wirnik_right_end2[2] = { wirnik_center2[0] + wirnik_length / 2 * cos(wirnik_theta), wirnik_center2[1] + wirnik_length / 2 * sin(wirnik_theta) };

    thickLineColor(gRenderer.get(), qlewo[0], qlewo[1], qprawo[0], qprawo[1], qheight, qcolor);

    thickLineColor(gRenderer.get(), lewykij1[0], lewykij1[1], lewykij2[0], lewykij2[1], kijWidth, kijkolor);
    thickLineColor(gRenderer.get(), prawykij1[0], prawykij1[1], prawykij2[0], prawykij2[1], kijWidth, kijkolor);

    unsigned int kolorki[3] = { 0xFFFF00FF, 0xF000F0FF, 0xF0F0F0F0 };
    int t = SDL_GetTicks();
    int ani = (t / 100) % 3;
    filledCircleColor(gRenderer.get(), q_x, q_y, qheight / 2, kolorki[ani]);

    int wirnik_color = 0xFFFFD7FF;
    int wirnik_width = 7;
    thickLineColor(gRenderer.get(), wirnik_left_end[0], wirnik_left_end[1], wirnik_right_end[0], wirnik_right_end[1], wirnik_width, wirnik_color);
    thickLineColor(gRenderer.get(), wirnik_left_end2[0], wirnik_left_end2[1], wirnik_right_end2[0], wirnik_right_end2[1], wirnik_width, wirnik_color);
}