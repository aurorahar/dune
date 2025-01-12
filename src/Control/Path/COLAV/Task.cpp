//***************************************************************************
// Copyright 2007-2022 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Faculdade de Engenharia da             *
// Universidade do Porto. For licensing terms, conditions, and further      *
// information contact lsts@fe.up.pt.                                       *
//                                                                          *
// Modified European Union Public Licence - EUPL v.1.1 Usage                *
// Alternatively, this file may be used under the terms of the Modified     *
// EUPL, Version 1.1 only (the "Licence"), appearing in the file LICENCE.md *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// https://github.com/LSTS/dune/blob/master/LICENCE.md and                  *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: Aurorahar                                                        *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Control
{
  namespace Path
  {
    namespace COLAV
    {
      using DUNE_NAMESPACES;

      struct Arguments{
        double dsep;
        double dsafe;
        double asafe;
        double frequency;
        double d_margin;
        double weight;
        double los_Delta;
        double timeout;
        bool follow_path;
      };

      struct Point{
        double x;
        double y;
      };

      struct Task: public DUNE::Control::PathController
      {
        Arguments m_args;
        IMC::DesiredHeading m_heading;

        //! Collision avoidance.
        double m_coll_cone[2];
        double m_alpha_min;
        double m_d_min;
        bool m_ca_active;
        bool m_wait_for_speed;
        int m_turn_dir;

        //! Obstacle state.
        IMC::Target m_ob;                 // Obstacle state
        Point m_ooffsett;                 // NE offset
        double m_or;                      // Estimated heading rate
        double m_ts;                      //! Simulation time step
        double m_time_last_received;      //! Time the last update was received
        bool m_os_received;               //! If we have valid obstacle measurements

        //! List to save the polygon vertices.
        IMC::MessageList<IMC::MapPoint> m_vertices;

        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Control::PathController(name, ctx),
          m_os_received(false),
          m_ca_active(false),
          m_wait_for_speed(false),
          m_time_last_received(-1.0)
        {
          param("Separation Distance", m_args.dsep)
          .description("Minimum distance the vehicle should keep to the obstacle.")
          .defaultValue("10.0")
          .units(Units::Meter);
          param("Safety Distance", m_args.dsafe)
          .description("Distance deciding when to initiate evasive maneuvers. ")
          .defaultValue("20.0")
          .units(Units::Meter);
          param("Safety Angle", m_args.asafe)
          .description("Safety angle by which we extend the collision cone.")
          .defaultValue("25.0")
          .units(Units::Degree);
          param("Measurement Frequency", m_args.frequency)
          .description("Frequency at which the obstacle state is updated.")
          .defaultValue("100.0");
          param("Distance Margin", m_args.d_margin)
          .description("Threshold value when checking if the distance is close to the safety distance.")
          .defaultValue("0.5")
          .units(Units::Meter);
          param("Weight", m_args.weight)
          .description("Weighting factor for choosing maneuver direction.")
          .defaultValue("0.5");
          param("Path Following", m_args.follow_path)
          .defaultValue("false");
          param("Look Ahead Distance", m_args.los_Delta)
          .defaultValue("5.0");
          param("COLAV Time Out",m_args.timeout )
          .description("Minimum time since last received measurement before obstacle is made inactive.")
          .defaultValue("5.0");

          bind<IMC::Target>(this);
          bind<IMC::MapFeature>(this);
        }

        void
        onUpdateParameters(void)
        {
          m_ts = 1.0/m_args.frequency;
          PathController::onUpdateParameters();
        }

        void
        onEntityReservation(void)
        {
          PathController::onEntityReservation();
        }

        void
        onPathActivation(void)
        {
          enableControlLoops(IMC::CL_YAW);
        }
        void
        onPathDeactivation(void)
        {
          disableControlLoops(IMC::CL_YAW);
        }

        //! Save position of polygon vertices.
        void consume(const IMC::MapFeature * msg){
          m_vertices = msg->feature;
        }

        //! Save obstacle measurements.
        void consume(const IMC::Target * msg){

         if(!m_os_received){
            m_os_received = true;
            m_or = 0.0;
          }
          else
            m_or = (msg->cog-m_ob.cog)/(m_ts);


          m_time_last_received = Clock::get();
          m_ob = *msg;
        }

        //! Computes the distance from the vehicle to the obstacle polygon.
        void
        compMinDistance(const IMC::EstimatedState & vs){

          double dir, d, alpha;
          Point p1, p2, p2_rot, vehicle_rot;

          //! Last vertex
          IMC::MapPoint * map1 = *(m_vertices.end()-1);

          for (IMC::MapPoint * map2 : m_vertices){
            Coordinates::WGS84::displacement(vs.lat, vs.lon, vs.height, map1->lat, map1->lon, vs.height, &p1.x, &p1.y);
            Coordinates::WGS84::displacement(vs.lat, vs.lon, vs.height, map2->lat, map2->lon, vs.height, &p2.x, &p2.y);

            //! Rotate vehicle pos to reference frame in p1 aligned with the line p1-p2.
            Coordinates::getBearingAndRange(p1, p2, &dir, &p2_rot.x);

            vehicle_rot.x = (vs.x-p1.x)*std::cos(-dir)-(vs.y-p1.y)*std::sin(-dir);
            vehicle_rot.y = (vs.x-p1.x)*std::sin(-dir)+(vs.y-p1.y)*std::cos(-dir);

            //! Then a simple check to see if vehicle pos is between 0 and p2.
            if (vehicle_rot.x <= 0 || vehicle_rot.x >= p2_rot.x)
              d = std::min(Coordinates::getRange(vs, p1), Coordinates::getRange(vs, p2));
            else
              d = Math::norm(vehicle_rot.x,vehicle_rot.y)*std::sin(std::abs(std::atan2(vehicle_rot.y, vehicle_rot.x)));

            alpha = Coordinates::getBearing(vs, p2);

            //! Update the minimum distance and orientation accordingly.
            m_alpha_min = std::min(alpha, m_alpha_min);
            m_d_min = std::min(d, m_d_min);

            map1 = map2;
          }
        }

        //! Computes the collision cone between the vehicle and the obstacle polygon.
        void
        compColCone(const EstimatedState & vs){

          double min_angle = +2*c_pi;
          double max_angle = -2*c_pi;
          double phi, alpha, d;

          double speed = Math::norm(vs.u, vs.v);
          Point p;


          for (IMC::MapPoint * map_p : m_vertices){

            Coordinates::WGS84::displacement(vs.lat, vs.lon, vs.height, map_p->lat, map_p->lon, vs.height, &p.x, &p.y);
            Coordinates::getBearingAndRange(m_ooffsett, p, &phi, &d);

            double vx = m_ob.sog*std::cos(m_ob.cog) - m_or*d*std::sin(phi);
            double vy = m_ob.sog*std::sin(m_ob.cog) + m_or*d*std::cos(phi);

            Coordinates::getBearingAndRange(vs, p, &alpha, &d);

            double beta = d >= m_args.dsep ? std::asin(m_args.dsep/d) : c_pi-std::asin(d/m_args.dsep);
            double ang_m = compAngle(alpha - beta - Angles::radians(m_args.asafe), vx, vy, speed);
            double ang_p = compAngle(alpha + beta + Angles::radians(m_args.asafe), vx, vy, speed);

            //! Feasibility check.

            if ( std::max(std::abs(ang_m),std::abs(ang_p)) > 1 ){
              debug("Cannot compute velocity compensation term -- speed too low.");
              m_wait_for_speed = true;
              return;
            }
            m_wait_for_speed = false;

            double rot_angle;

            if ((rot_angle = Angles::normalizeRadian(alpha-m_alpha_min)-beta-Angles::radians(m_args.asafe) + std::asin(ang_m)) < min_angle){
              min_angle = rot_angle;
              m_coll_cone[0] = mapAngle(alpha - beta - Angles::radians(m_args.asafe) + std::asin(ang_m));
            }

            if ((rot_angle = Angles::normalizeRadian(alpha-m_alpha_min)+beta+Angles::radians(m_args.asafe) + std::asin(ang_p)) > max_angle){
              max_angle = rot_angle;
              m_coll_cone[1] = mapAngle(alpha + beta + Angles::radians(m_args.asafe) + std::asin(ang_p));
            }
          }
        }

        double
        compAngle(double psic, double vx, double vy, double speed){
           return  ( Math::norm(vx,vy)/speed * std::sin(c_pi-std::atan2(vy,vx)+psic) );
        }

        //! Returns the angle modulated between 0 and 2pi.
        double
        mapAngle(double angle){
          return angle - 2.0*c_pi * std::floor(angle/(2.0*c_pi));
        }


        void
        shouldCA(const IMC::EstimatedState & vs, double course){

          Coordinates::WGS84::displacement(vs.lat, vs.lon, vs.height, m_ob.lat, m_ob.lon, vs.height, &m_ooffsett.x, &m_ooffsett.y);
          Coordinates::getBearingAndRange(vs, m_ooffsett, &m_alpha_min, &m_d_min);

          compMinDistance(vs);

          // ! Collision avoidance algorithm.
          // ! Check if the vehicle is already avoiding collision or if the
          // ! distance to the obstacle has been reduced to less than the safety
          // ! distance. If so, check if the current course is in the interior
          // ! of the collision cone to activate collision avoidance control.
          // ! If we are not in an ongoing evasive maneuver, but are initiating
          // ! one, we choose the turning direction.

          if ( m_d_min <= m_args.dsafe || m_ca_active ){
            compColCone(vs);

            if ( m_wait_for_speed ){
              m_ca_active = false;
              return;
            }

            //! Check if the desired course is between the collision cone angles.

            double direction_rot = mapAngle(mapAngle(course) - m_coll_cone[0]);
            double cone_rot = mapAngle(m_coll_cone[1] - m_coll_cone[0]);

            if ( direction_rot < cone_rot ) {

              //! Here we choose turning direction. We use a weighting factor to choose between the
              //! course that will take the vehicle behind the obstacle and the course that is closest to
              //! the current course angle.

              if ( !m_ca_active ){
                if ( m_args.dsafe-m_d_min < m_args.d_margin ){

                  int closest = std::abs(Angles::minSignedAngle(course,m_coll_cone[0])) <= std::abs(Angles::minSignedAngle(course,m_coll_cone[1])) ? 0 : 1;
                  int behind =  std::abs(Angles::minSignedAngle(m_ob.cog,m_coll_cone[0])) >= std::abs(Angles::minSignedAngle(m_ob.cog,m_coll_cone[1])) ? 0 : 1;

                  m_turn_dir = std::abs(Angles::minSignedAngle(course,m_coll_cone[closest])) <= m_args.weight*std::abs(Angles::minSignedAngle(course,m_coll_cone[behind])) ? closest : behind;
                }
                else
                  m_turn_dir = std::abs(Angles::minSignedAngle(course,m_coll_cone[0])) <= std::abs(Angles::minSignedAngle(course,m_coll_cone[1])) ? 0 : 1;

                debug("Avoiding collision.");
              }
              //! Activate collision avoidance.
              m_ca_active = true;
            }
            else{
              if ( m_ca_active )
                debug("Not avoiding collision.");
              //! Deactivate collision avoidance.
              m_ca_active = false;
            }
          }

        }


        void
        step(const IMC::EstimatedState & state, const TrackingState& ts)
        {
          m_heading.value = m_args.follow_path ? ts.track_bearing + std::atan2(-ts.track_pos.y,m_args.los_Delta) : ts.los_angle;

          if (m_os_received && Clock::get()- m_time_last_received >= m_args.timeout){
            m_os_received = false;
          }

          if (m_os_received){

            //! Check if we need to avoid a collision.
            shouldCA(state, m_heading.value);

            if ( m_ca_active )
              m_heading.value = m_coll_cone[m_turn_dir];
          }

          //! If we are controlling the course, compensate for the crab angle.
          if ( ts.cc )
            m_heading.value = Angles::normalizeRadian(m_heading.value + state.psi - ts.course);

          dispatch(m_heading);
        }

      };
    }
  }
}

DUNE_TASK
