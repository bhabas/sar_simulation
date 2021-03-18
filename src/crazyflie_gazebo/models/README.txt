Notes:
    - ERP set to 0.8 under joint to pullback detached links if they drift away or go flying off

    - CFM is how hard a constraint is forced. Higher CFM makes a softer constrain and less realistic numbers but also increases stability. There's a tradeoff and balance here

    - Gazebo will sometimes overwrite your changes for inertia or other parameters when you leave the model editor. Always double your SDF file. Use the model editor for placements but the SDF for the finer details of the physics





Open Dynamics Engine (ODE): Physics engine we are running currently. Userguide: [http://www.ode.org/ode-latest-userguide.html#sec_7_3_5]

Constrain Forcing Matrix (CFM)

    - Most constraints are by nature ``hard''. This means that the constraints represent conditions that are never violated. For example, the ball must always be in the socket, and the two parts of the hinge must always be lined up. In practice constraints can be violated by unintentional introduction of errors into the system, but the error reduction parameter can be set to correct these errors.

    Not all constraints are hard. Some ``soft'' constraints are designed to be violated. For example, the contact constraint that prevents colliding objects from penetrating is hard by default, so it acts as though the colliding surfaces are made of steel. But it can be made into a soft constraint to simulate softer materials, thereby allowing some natural penetration of the two objects when they are forced together.

    ERP and CFM can be independently set in many joints. They can be set in contact joints, in joint limits and various other places, to control the spongyness and springyness of the joint (or joint limit).

    If CFM is set to zero, the constraint will be hard. If CFM is set to a positive value, it will be possible to violate the constraint by ``pushing on it'' (for example, for contact constraints by forcing the two contacting objects together). In other words the constraint will be soft, and the softness will increase as CFM increases. What is actually happening here is that the constraint is allowed to be violated by an amount proportional to CFM times the restoring force that is needed to enforce the constraint. Note that setting CFM to a negative value can have undesirable bad effects, such as instability. Don't do it.


Error Reduction Parameter (ERP)

    When a joint attaches two bodies, those bodies are required to have certain positions and orientations relative to each other. However, it is possible for the bodies to be in positions where the joint constraints are not met. This ``joint error'' can happen in two ways:

        If the user sets the position/orientation of one body without correctly setting the position/orientation of the other body.
        During the simulation, errors can creep in that result in the bodies drifting away from their required positions. 

    There is a mechanism to reduce joint error: during each simulation step each joint applies a special force to bring its bodies back into correct alignment. This force is controlled by the error reduction parameter (ERP), which has a value between 0 and 1.

    The ERP specifies what proportion of the joint error will be fixed during the next simulation step. If ERP=0 then no correcting force is applied and the bodies will eventually drift apart as the simulation proceeds. If ERP=1 then the simulation will attempt to fix all joint error during the next time step. However, setting ERP=1 is not recommended, as the joint error will not be completely fixed due to various internal approximations. A value of ERP=0.1 to 0.8 is recommended (0.2 is the default).

    A global ERP value can be set that affects most joints in the simulation. However some joints have local ERP values that control various aspects of the joint.