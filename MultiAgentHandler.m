classdef MultiAgentHandler < handle
    properties (SetAccess = protected)
        agents_ % Object array of AgentHnadlerEstimate class
    end
    properties (SetAccess = immutable)
        num_agents
        num_dims
    end

    methods (Access = public)
        function obj = MultiAgentHandler(args, initial_states_estimate)
            num_agents  = args.num_agents;
            num_dims    = args.num_dimensions;
            args_agent.position = zeros(num_dims,1);
            args_agent.velocity = zeros(num_dims,1);
            for iAgents = 1:num_agents
                args_agent.id = iAgents;
                agents_(iAgents) = AgentHandler(args_agent);
            end
            obj.num_agents  = num_agents;
            obj.num_dims    = num_dims;
            obj.agents_     = agents_;
            for iAgents = 1:num_agents
                obj.setInitialEstimates(initial_states_estimate);
            end
        end

        % Setters
        function setInitialEstimates(this, initial_states_estimate)
            num_agents = this.num_agents;
            num_dims = this.num_dims;
            for iAgents = 1:this.num_agents
                this.agents_(iAgents).setPosition(initial_states_estimate(1:num_dims,iAgents));
                this.agents_(iAgents).setVelocity(initial_states_estimate(num_dims+1:2*num_dims,iAgents));
            end
        end
        function setAgentPositionVelocity(this, iAgents, posvel)
            num_dims = this.num_dims;
            this.agents_(iAgents).setPositionVelocity(posvel);
        end
        function setStateVector(this, state_vector)
            num_agents = this.num_agents;
            num_vars = 2 * this.num_dims;
            for iAgents = 1:num_agents
                posvel = state_vector(num_vars*(iAgents-1)+1 : num_vars*iAgents, 1);
                this.agents_(iAgents).setPositionVelocity(posvel);
            end
        end

        % Getters
        function output = getStateVectorOfAllAgents(this)
            num_vars = 2*this.num_dims;
            output = zeros(num_vars*this.num_agents,1);
            for iAgents = 1:this.num_agents
                posvel = this.getAgentPositionVelocity(iAgents);
                output(num_vars*(iAgents-1)+1:num_vars*iAgents, 1) = posvel;
            end
        end

        function output = getAgentPositionVelocity(this, iAgents)
            position = this.getAgentPosition(iAgents);
            velocity = this.getAgentVelocity(iAgents);
            output = vertcat(position, velocity);
        end
        function output = getAgentPosition(this, iAgents)
            output = this.agents_(iAgents).getPosition();
        end
        function output = getAgentVelocity(this, iAgents)
            output = this.agents_(iAgents).getVelocity();
        end

    end
end