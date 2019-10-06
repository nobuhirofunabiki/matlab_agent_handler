classdef AgentEstimate < Agent
    properties (SetAccess = private)
        covmat_pos_vel      % position and velocity 
        covmat_attitude     % quaternion and angular velocity
        pre_position
    end

    methods
        function obj = AgentEstimate(args, args_est)
            obj@Agent(args);

            for itr = 1:numel(args.position)
                this.covmat_pos_vel(itr,itr) = args_est.sigma_position^2;
            end
            for itr = numel(args.position)+1:numel(args.position)+numel(args.velocity)
                this.covmat_pos_vel(itr,itr) = args_est.sigma_velocity^2;
            end
            obj.pre_position = obj.position;
        end

        function propagateCovarianceMatrix(this, control_input, dynamics_)
            Ad = dynamics_.getDiscreteSystemMatrix;
            Bc = dynamics_.getDiscreteInputMatrix;
            P_pri = this.covmat_pos_vel;
            % TODO: Add control and disturbance noises
            this.covmat_pos_vel = Ad*P_pri*Ad';
        end

        function est_velocity = estimateVelocitySimple(this, delta_time)
            this.velocity = (this.position - this.pre_position)/delta_time;
        end

        function setIsotropicStateCovariance(this, sigma_pos, sigma_vel)
            for itr = 1:3
                this.covmat_pos_vel(itr,itr) = sigma_pos^2;
            end
            for itr = 4:6
                this.covmat_pos_vel(itr,itr) = sigma_vel^2;
            end
        end

        function setPreviousPosition(this)
            this.pre_position = this.position;
        end

    end
end