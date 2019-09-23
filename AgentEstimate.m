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
        end

        function propagateCovarianceMatrix(this, control_input, dynamics_)
            Ad = dynamics_.getDiscreteSystemMatrix;
            Bc = dynamics_.getDiscreteInputMatrix;
            P_pri = this.covmat_pos_vel;
            % TODO: Add control and disturbance noises
            this.covmat_pos_vel = Ad*P_pri*Ad';
        end

    end
end