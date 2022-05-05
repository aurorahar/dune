
// DUNE headers.
#include <DUNE/DUNE.hpp>
#include <cmath>


namespace Testing
{
  //! Obstacle simulator.
  //!
  //! Task to simulate a moving obstacle and transmit the position and
  //! velocity to the vehicle. Collision avoidance implemented in the path
  //! controller.
  //!
  //! ***Currently testing sending maneuvers to the vehicle from this task**

  //! @author Aurora
  namespace OSIM
  {
    using DUNE_NAMESPACES;

    //! Obstacle State
    //! Lat, lon : Latidual and longitudal coordinates.
    //! x, y     : North and East offsets from the inertial origin.
    //! psi      : Rotation over x axis
    //! u        : Body-fixed x velocity
    //! vx       : Ground x velocity
    //! vy       : Ground y velocity

    //! We do not model any other states, these can hence be considered 0.

    struct obstacle{
      double lat;
      double lon;
      double psi;
      double x;
      double y;
      double u;
      double vx;
      double vy;
    };

    struct Arguments{
      std::vector<double> position;
      std::vector<double> offset;
      double heading;
      double speed;
      double r_max;
      double a_max;
      double u_max;
    };

    struct Task: public DUNE::Tasks::Periodic
    {

      //! Arguments.
      Arguments args;

      //! Obstacle state.
      Obstacle os;

      //! Maneuver to be requested.
      IMC::Goto man_goto;
      InlineMessage<Maneuver> m_man;

      //! Command to be sent to the vehicle.
      IMC::VehicleCommand m_command;
      const unint_16_t id = 100;

      //! Time interval for numerical integration.
      double ts;

      // Handling request timeout.
      bool m_waiting_response;
      bool m_executing;
      double c_req_time;

      static const float c_req_timeout = 10;



      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Periodic(name, ctx),
        m_waiting_response(false),
        m_executing(false)
      {
        param("Initial Position", args.position)
        .description("Initial position of the obstacle.")
        .size(2)
        .units(Units::Degree);

        param("Initial Offset", args.offset)
        .description("Initial x and y positions.")
        .size(2)
        .units(Units::Meter);

        param("Heading", args.heading)
        .description("Initial orientation of the obstacle.")
        .defaultValue("0.0")
        .units(Units::Degree);

        param("Speed", args.speed)
        .description("Initial speed of the obstacle.")
        .defaultValue("0.0")
        .units(Units::MeterPerSecond);

         param("Maximum Heading Rate", args.r_max)
        .description("Maximum obstacle heading rate.")
        .defaultValue("0.0")
        .units(Units::DegreePerSecond); // should be radianspersec

        param("Maximum Speed", args.u_max)
        .description("Maximum obstacle speed.")
        .defaultValue("0.0")
        .units(Units::MeterPerSecond);

        param("Maximum Acceleration", args.a_max)
        .description("Maximum obstacle acceleration.")
        .defaultValue("0.0")
        .units(Units::MeterPerSquareSecond);


        bind<IMC::VehicleCommand>(this);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        //! Set time step according to the execution frequency.
        ts = 1.0/getFrequency();

        //! Obstacle state.
        os.lat = Math::Angles::radians(args.position[0]);
        os.lon = Math::Angles::radians(args.position[1]);
        os.x = args.offset[0];
        os.y = args.offset[1];
        os.psi = args.heading;
        os.u = args.speed;
        os.vx = os.u*std::cos(os.psi);
        os.vy = os.u*std::sin(os.psi);

        //! Set origin of the maneuver
        man_goto.lat =Math::Angles::radians(args.position[0]);
        man_goto.lon = Math::Angles::radians(args.position[1]);

        //! Set the desired position an offset of 30.0 meters in North and East directions.
        //! This is chosen arbitrary just to test the path controller.

        WGS84::displace(30.0, 30.0,  &man_goto.lat, &man_goto.lon);
        man_goto.z = 0.0; // We are at the surface.
        man_goto.z_units = Units::Meter;

        //! Inline message.
        m_man.set(&man_goto);

        //! Set the command attributes, these are necessary for the command to be accepted.
        //! We request execution of a maneuver.
        m_command.maneuver = m_man;
        m_command.type = IMC::VehicleCommand::VC_REQUEST;
        m_command.command = IMC::VehicleCommand::VC_EXEC_MANEUVER;
        m_command.request_id = id;

        //! Note that, if external control is enabled, maneuvers can be transmitted directly to the vehicle,
        //! and do not have to be requested by sending a command as we do here.
      }

      //! Reserve entity identifiers.
      void
      onEntityReservation(void)
      {
        inf("Starting: %s", resolveEntity(getEntityId()).c_str());
      }

      //! Resolve entity names.
      void
      onEntityResolution(void)
      {
      }

      //! Acquire resources.
      void
      onResourceAcquisition(void)
      {
      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
      }

      void consume(IMC::VehicleCommand * reply_msg){
        if ! ( (reply_msg->request_id == m_command.request_id) && (reply_msg->command == m_command.command) ) {
          inf("Request ids and/or commands do not match. Returning. ")
          return;
        }

        if (reply_msg->type == IMC::VehicleCommand::VC_SUCCESS){
          inf("Received message: %s", reply_msg->info);
          m_executing = true;
          return;
        }

        if (reply_msg->type == IMC::VehicleCommand::VC_FAILURE){
          inf("Received message: %s", reply_msg->info);
          m_executing = false;
          return;
        }
      }

      //! Updates the position and velocity of the obstacle.

      void updatePosVel(double headingRate, double acceleration){

        //! Unicycle kinematics.
        os.x += os.u*std::cos(os.psi)*ts;
        os.y += os.u*std::sin(os.psi)*ts;
        os.psi += std::max(std::min(headingRate,args.r_max), -args.r_max)*ts;
        double u = os.u+std::max(std::min(acceleration,args.a_max), -args.a_max)*ts;

        //! Ensure that speed is between 0 and umax.
        os.u = std::max(std::min(u, args.u_max), 0.0);

        //! Update ground velocity.
        os.vx = os.u*std::cos(os.psi);
        os.vy = os.u*std::sin(os.psi);

        //! Update longitudal and Latidual coordinates.
        WGS84::displace(os.x, os.y,  &os.lat, &os.lon);
      }

      //! Proportional heading controller.

      double headingController(double desiredHeading){
        return Angles::normalizeRadian(- os.psi + desiredHeading);
      }

      //! Proportional speed controller.

      double speedController(double desiredSpeed){
        return - os.u + desiredSpeed;
      }



      //! Main loop.
      void
      task(void)
      {

        if !(m_waiting_response){
          m_waiting_response = true;
          c_req_time = Clock::get();

          dispatch(m_command);
          inf("Requested maneuver 'Goto (%f, %f)'. ", man_goto.lat, man_goto.lon);
        }


        double c_time = Clock_get();
        if (!(m_executing) || c_time - c_trans_time >=c_req_timeout){
            inf("Request timed out. ");
            m_waiting_response = false;
        }






        // Testing obstacle
        //updatePosVel(headingController( c_pi/2.0), speedController(3.0));

        //inf("New coordinates are (%f, %f)", os.x, os.y);

        //! TODO: transmit obstacle state
        //! Should it be converted somehow?



      }
    };
  }
}

DUNE_TASK
