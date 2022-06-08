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
        double d_margin;
        double weight;
        double los_Delta;
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

        //! Collision Avoidance
        double m_coll_cone[2];
        bool m_ca_active;
        int m_turn_dir;

        //! Obstacle State
        Point m_ooffsett;
        IMC::Target m_ob;
        bool m_os_received;

        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Control::PathController(name, ctx),
          m_os_received(false),
          m_ca_active(false)
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
          .defaultValue("10.0")
          .units(Units::Degree);
          param("Distance Margin", m_args.d_margin)
          .defaultValue("0.1")
          .units(Units::Meter);
          param("Weight",m_args.weight)
          .defaultValue("0.5");
          param("Path Following", m_args.follow_path)
          .defaultValue("false");
          param("Look Ahead Distance", m_args.los_Delta)
          .defaultValue("5.0");

          bind<IMC::Target>(this);
        }

        void
        onUpdateParameters(void)
        {
          m_args.asafe = Angles::radians(m_args.asafe);
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

        void consume(const IMC::Target * os){
          m_ob = * os;

          if (!m_os_received)
            m_os_received = true;

        }


        //! Returns the angle modulated between 0 and 2pi.
        double
        mapAngle(double angle){
          return angle - 2.0*c_pi * std::floor(angle/(2.0*c_pi));
        }


        void
        shouldCA(const IMC::EstimatedState & vs, double course){

          //! Compute x,y offset of obstacle
          Coordinates::WGS84::displacement(vs.lat, vs.lon, vs.height, m_ob.lat, m_ob.lon, vs.height, &m_ooffsett.x, &m_ooffsett.y);

          //! Compute the distance and orientation to the obstacle.
          double d, alpha, beta;
          Coordinates::getBearingAndRange(vs, m_ooffsett, &alpha, &d);

          if( d < m_args.dsep ){
            //! Should not happen but if we are inside the collision region.
            debug("Distance to obstacle is %f [m] -- Inside collision region", d);
            beta = c_pi - std::asin(d/m_args.dsep);
          } else {
            beta = std::asin(m_args.dsep/d);
          }

          //! Collision avoidance algorithm.
          //! Check if the vehicle is already avoiding collision or if the
          //! distance to the obstacle has been reduced to less than the safety
          //! distance. If so, check if the current course is in the interior
          //! of the collision cone to activate collision avoidance control.
          //! If we are not in an ongoing evasive maneuver, but are initiating
          //! one, we choose the turning direction.

          if ( d <= m_args.dsafe || m_ca_active ){

            double speed = Math::norm(vs.u, vs.v);
            double ang_m =  std::sin(c_pi - m_ob.cog + alpha - beta - m_args.asafe);
            double ang_p =  std::sin(c_pi - m_ob.cog + alpha + beta + m_args.asafe);

            //! Feasibility check
            if (std::max(std::abs(ang_p),std::abs(ang_m))*m_ob.sog > speed){
              //! Cannot compute the velocity compensation term, resuming guidance.
              debug("collision cone error");
              m_ca_active = false;
              return;
            }

            //! Compute cone angles modulated between 0 and 2pi. This mapping
            //! ensures that the collision check below is correct.
            m_coll_cone[0] = mapAngle(alpha - beta - m_args.asafe + std::asin(m_ob.sog/speed * ang_m)) ;
            m_coll_cone[1] = mapAngle(alpha + beta + m_args.asafe + std::asin(m_ob.sog/speed * ang_p)) ;

            //! Check if the desired course is between the collision cone angles.
            double direction_rot = mapAngle(mapAngle(course) - m_coll_cone[0]);
            double cone_rot = mapAngle(m_coll_cone[1] - m_coll_cone[0]);

            if ( direction_rot < cone_rot ) {

              //! Here we choose turning direction.
              if ( !m_ca_active ){
                if(m_args.dsafe - d < m_args.d_margin){
                  int closest = std::abs(Angles::minSignedAngle(course,m_coll_cone[0]))<= std::abs(Angles::minSignedAngle(course,m_coll_cone[1])) ? 0 : 1;
                  int behind =  std::abs(Angles::minSignedAngle(m_ob.cog,m_coll_cone[0]))>= std::abs(Angles::minSignedAngle(m_ob.cog,m_coll_cone[1])) ? 0 : 1;

                  m_turn_dir = std::abs(Angles::minSignedAngle(course,m_coll_cone[closest])) <= m_args.weight*std::abs(Angles::minSignedAngle(course,m_coll_cone[behind])) ? closest : behind;

                }
                else
                  m_turn_dir = std::abs(Angles::minSignedAngle(course,m_coll_cone[0])) <= std::abs(Angles::minSignedAngle(course,m_coll_cone[1])) ? 0 : 1;

                inf("Avoiding collision.");
              }
              //! Activate collision avoidance.
              m_ca_active = true;

            }
            else{

              if ( m_ca_active )
                inf("Not avoiding collision.");
              //! Deactivate collision avoidance.
              m_ca_active = false;

            }
          }
        }

        void
        step(const IMC::EstimatedState & state, const TrackingState& ts)
        {
          m_heading.value = m_args.follow_path ? ts.track_bearing+std::atan2(-ts.track_pos.y,m_args.los_Delta) : ts.los_angle;

          if (m_os_received){
            //! Check if we need to avoid a collision
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
