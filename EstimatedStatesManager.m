classdef EstimatedStatesManager
    properties (SetAccess = protected)
        agents_est % Object array of AgentEstimate class
        num_agents
        num_dims
    end
    methods
        function obj = EstimatedStatesManager(args)
            num_agents = args.num_agents;
            num_dims = args.num_dimensions;
            memory_size = args.memory_size;
            sigma_position = args.sigma_position;
            sigma_velocity = args.sigma_velocity;

            args_est1.position = zeros(num_dims,1);
            args_est1.velocity = zeros(num_dims,1);
            args_est1.memory_size = memory_size;
            args_est2.sigma_position = sigma_position;
            args_est2.sigma_velocity = sigma_velocity;
            for iAgents = 1:num_agents
                args_est1.id = iAgents;
                agents_est(iAgents) = AgentEstimate(args_est1, args_est2);
            end
            obj.num_agents = num_agents;
            obj.num_dims = num_dims;
            obj.agents_est = agents_est;
        end

        % Setters
        function setInitialEstimates(this, initial_states_estimate)
            num_agents = this.num_agents;
            num_dims = this.num_dims;
            for iAgents = 1:this.num_agents
                this.agents_est(iAgents).setPosition(initial_states_estimate(1:num_dims,iAgents));
                this.agents_est(iAgents).setVelocity(initial_states_estimate(num_dims+1:2*num_dims,iAgents));
            end
        end
        function setAgentPositionVelocity(this, iAgents, posvel)
            num_dims = this.num_dims;
            this.agents_est(iAgents).setPositionVelocity(posvel);
        end
        function setStateVector(this, state_vector)
            num_agents = this.num_agents;
            num_vars = 2 * this.num_dims;
            for iAgents = 1:num_agents
                posvel = state_vector(num_vars*(iAgents-1)+1 : num_vars*iAgents, 1);
                this.agents_est(iAgents).setPositionVelocity(posvel);
            end
        end

        % Getters
        function output = getAgentPositionVelocity(this, iAgents)
            position = this.getAgentPosition(iAgents);
            velocity = this.getAgentVelocity(iAgents);
            output = vertcat(position, velocity);
        end
        function output = getAgentPosition(this, iAgents)
            output = this.agents_est(iAgents).getPosition();
        end
        function output = getAgentVelocity(this, iAgents)
            output = this.agents_est(iAgents).getVelocity();
        end

    end
end