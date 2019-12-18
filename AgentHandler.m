classdef AgentHandler < handle
    properties (SetAccess = protected)
        id
        position
        velocity
        attitude
        angular_velocity
    end

    methods
        function obj = AgentHandler(args)
            obj.id = args.id;
            obj.position = args.position;
            obj.velocity = args.velocity;
            % obj.attitude = args.attitude;
            % obj.angular_velocity = args.angular_velocity;
        end

        function propagatePosition(this, control_input, delta_t, dynamics)
            Ac = dynamics.getSystemMatrix;
            Bc = dynamics.getInputMatrix;
            sigma = dynamics.getDisturbance;
            x = rungeKutta4Position(this.position, this.velocity, control_input, delta_t, Ac, Bc, sigma);
            this.position = x(1:3,:);
            this.velocity = x(4:6,:);
        end
        
        % Get functions
        function obj = getInterAgentsDistance(this1, this2)
            obj = norm(this2.position - this1.position);
        end
        function obj = getID(this)
            obj = this.id;
        end
        function obj = getPosition(this)
            obj = this.position;
        end
        function obj = getVelocity(this)
            obj = this.velocity;
        end
        function obj = getPositionVelocity(this)
            obj = vertcat(this.position, this.velocity);
        end
        function obj = getAttitude(this)
            obj = this.attitude;
        end
        function obj = getAngularVelocity(this)
            obj = this.angular_velocity;
        end

        % Set functions
        function setPosition(this, arg_position)
            this.position = arg_position;
        end
        function setVelocity(this, arg_velocity)
            this.velocity = arg_velocity;
        end
        function setPositionVelocity(this, arg_posvel)
            dim_pos = numel(this.position);
            dim_vel = numel(this.velocity);
            this.position = arg_posvel(1:dim_pos,1);
            this.velocity = arg_posvel(dim_pos+1:dim_pos+dim_vel,1);
        end
    end
end