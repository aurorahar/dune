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
  //! Pure pursuit path controller with collision avoidance.
  //!
  //! @author Aurorahar
  //! Original pure pursuit controller by Eduardo Marques

  namespace Path
  {
    namespace COLAV_PurePursuit
    {
      using DUNE_NAMESPACES;

      struct Arguments{
        double dsep;
        double dsafe;
        double asafe;
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
        double m_opos[2];
        double m_opsi;
        double m_ou;
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
          //! Save obstacle measurements.
          m_opos[0] = os->lat;
          m_opos[1] = os->lon;
          m_opsi = os->cog;
          m_ou = os->sog;

          if (!m_os_received){
            m_os_received = true;
          }
        }

        //! Returns the angle modulated between 0 and 2pi.
        double mapAngle(double angle){
          return angle - 2.0*c_pi * std::floor(angle/(2.0*c_pi));
        }


        void shouldCA(const IMC::EstimatedState & vs, double course){

          //! Compute the latitudinal and longitudinal coordinates of the vehicle.
          double la = vs.lat;
          double lo = vs.lon;
          Coordinates::WGS84::displace(vs.x, vs.y, &la, &lo);

          //! Compute the distance and orientation to the obstacle.
          double d, alpha, beta;
          Coordinates::WGS84::getNEBearingAndRange(la, lo, m_opos[0], m_opos[1], & alpha, & d);

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

            double u = vs.u;
            double v = vs.v;

            double speed = std::sqrt(u*u + v*v);

            //! Feasibility check
            if (m_ou > speed){
              //! Cannot compute the velocity compensation term.
              debug("collision cone error");
              m_ca_active = false;
              return;
            }

            //! Compute cone angles modulated between 0 and 2pi. This mapping
            //! ensures that the collision check below is correct.

            m_coll_cone[0] = mapAngle(alpha - beta - m_args.asafe + std::asin(m_ou/speed * std::sin(c_pi - m_opsi + alpha - beta - m_args.asafe))) ;
            m_coll_cone[1] = mapAngle(alpha + beta + m_args.asafe + std::asin(m_ou/speed * std::sin(c_pi - m_opsi + alpha + beta + m_args.asafe))) ;

            //! Check if the desired course is between the collision cone angles.
            double direction_rot = mapAngle(mapAngle(course) - m_coll_cone[0]);
            double cone_rot = mapAngle(m_coll_cone[1] - m_coll_cone[0]);

            if ( direction_rot < cone_rot ) {

              //! Here we choose turning direction.
              if ( !m_ca_active ){
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
          if (!m_os_received)
          //! If we have not received obstacle measurements, PP guidance.
            m_heading.value = ts.los_angle;
          else
          {
            //! Check if we need to avoid a collision
            shouldCA(state, ts.los_angle);

            if ( m_ca_active )
              m_heading.value = m_coll_cone[m_turn_dir];
            else
              m_heading.value = ts.los_angle;
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
