classdef AgentHandlerEstimate < AgentHandler
    properties (SetAccess = private)
        covmat_pos_vel      % position and velocity 
        covmat_attitude     % quaternion and angular velocity
        pre_position
    end

    methods
        function obj = AgentHandlerEstimate(args, args_est)
            obj@AgentHandler(args);
            dimension = obj.dimension;

            for iDims = 1:dimension
                obj.covmat_pos_vel(iDims,iDims) = args_est.sigma_position^2;
            end
            for iDims = dimension+1:2*dimension
                obj.covmat_pos_vel(iDims,iDims) = args_est.sigma_velocity^2;
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

        % Set functions
        function setStateCovariancePositionVelocity(this, arg_cov_mat)
            this.covmat_pos_vel = arg_cov_mat;
        end
        function setIsotropicStateCovariance(this, sigma_pos, sigma_vel)
            dimension = this.dimension;
            for iDims = 1:dimension
                this.covmat_pos_vel(iDims,iDims) = sigma_pos^2;
            end
            for iDims = dimension+1:2*dimension
                this.covmat_pos_vel(iDims,iDims) = sigma_vel^2;
            end
        end
        function setPreviousPosition(this)
            this.pre_position = this.position;
        end

        % Get functions
        function output = getStateCovariancePositionVelocity(this)
            output = this.covmat_pos_vel;
        end
    end
end