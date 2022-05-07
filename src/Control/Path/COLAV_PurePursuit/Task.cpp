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
//#include <cmath>

namespace Control
{
  //! Insert short task description here.
  //!
  //! Insert explanation on task behaviour here.
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

        Arguments args;
        IMC::DesiredHeading m_heading;

        //! Collision Avoidance
        double coll_cone[2];
        bool m_in_colreg;
        bool m_ca_active;
        int m_turn_dir;


        //! Obstacle State
        double opos[2];
        double opsi;
        double ou;
        bool m_os_received;


        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Control::PathController(name, ctx),
          m_os_received(false),
          m_in_colreg(false),
          m_ca_active(false)
        {
          param("Separation Distance", args.dsep)
          .defaultValue("10.0")
          .units(Units::Meter);
          param("Safety Distance", args.dsafe)
          .defaultValue("20.0")
          .units(Units::Meter);
          param("Safety Angle", args.asafe)
          .defaultValue("10.0")
          .units(Units::Degree);

          bind<IMC::Target>(this);
        }

        void
        onUpdateParameters(void)
        {
          args.asafe = Angles::radians(args.asafe);
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

          //! Save obstacle measurements
          opos[0] = os->lat;
          opos[1] = os->lon;
          opsi = os->cog;
          ou = os->sog;

          m_os_received = true;
        }

        void shouldCA(const IMC::EstimatedState& vs, const double course){

          double d, alpha, beta;
          Coordinates::WGS84::getNEBearingAndRange(vs.lat, vs.lon, opos[0], opos[1], & alpha, & d);

          m_in_colreg = d < args.dsep;

          //! Should not happen but if we are inside the collision region.

          if( m_in_colreg ){
            debug("Distance to obstacle is %f [m] -- Inside collision region", d);
            beta = c_pi - std::asin(d/args.dsep);
          } else {
            beta = std::asin(args.dsep/d);
          }

          double u = vs.u;
          double v = vs.v;

          double speed = std::sqrt(u*u + v*v);

          if (ou > speed){
            // Cannot compute the velocity compensation term.
            signalError(DTR("Collision cone error"));
            return;
          }

          //! Compute cone angles modulated between 0 and 2pi.

          coll_cone[0] = alpha - beta - args.asafe + std::asin(ou/speed * std::sin(c_pi - opsi + alpha - beta - args.asafe) ) ;
          coll_cone[1] = alpha + beta + args.asafe + std::asin(ou/speed * std::sin(c_pi - opsi + alpha + beta + args.asafe) ) ;

          //! Collision avoidance algorithm.
          //! We check if the vehicle is already avoiding collision or if the
          //! distance to the obstacle has been reduced to less than the safety
          //! distance. If so, we check if the current course is in the interior
          //! of the collision cone to activate collision avoidance control.
          //! If we are not in an ongoing evasive maneuver, but are initiating
          //! one, we choose the turning direction.

          if ( d <= args.dsafe || m_ca_active ){

            double direction_rot = fmod(course - coll_cone[0] , 2.0*c_pi);
            double cone_rot = fmod(coll_cone[1] - coll_cone[0] , 2.0*c_pi);

            if ( direction_rot < cone_rot ) {

              if ( !m_ca_active )
                m_turn_dir = std::abs(Angles::minSignedAngle(course,coll_cone[0])) <= std::abs(Angles::minSignedAngle(course,coll_cone[1])) ? 0 : 1;
              m_ca_active = true;
            }
            else{
              m_ca_active = false;
            }
          }
        }

        void
        step(const IMC::EstimatedState& state, const TrackingState& ts)
        {

          if (!m_os_received)
            m_heading.value = ts.los_angle;
          else
          {
            //! If we have received measuements of the obstacle, check if we
            //! need collision avoidance.

            shouldCA(state, ts.course);

            if ( m_ca_active )
              m_heading.value = coll_cone[m_turn_dir];
            else
              m_heading.value = ts.los_angle;
          }

          if ( ts.cc )
            m_heading.value = Angles::normalizeRadian(m_heading.value + state.psi - ts.course);

          dispatch(m_heading);
        }
// TODO
// Implement a timeout for received obstacle measurements?
      };
    }
  }
}

DUNE_TASK
