classdef TestAgentEstimate < matlab.unittest.TestCase
    properties
        agent_estimate_
    end

    methods (TestMethodSetup)
        function createAgentEstimate(testCase)
            args.id = 'test';
            args.position = [1; 2; 3];
            args.velocity = [0.1; 0.2; 0.3];
            args.memory_size = 1;
            % args.attitude = [0; 0; 0; 1];
            % args.angular_velocity = [0; 0; 0];
            args_est.sigma_position = 1.0;
            args_est.sigma_velocity = 1.0;
            testCase.agent_estimate_ = AgentEstimate(args, args_est);
        end
    end

    methods (Test)
        function testSetGetCovarianceMatrix(testCase)
            agent_estimate_ = testCase.agent_estimate_;
            covmat_expected = [...
                1 0 0 0 0 0;
                0 1 0 0 0 0;
                0 0 1 0 0 0;
                0 0 0 1 0 0;
                0 0 0 0 1 0;
                0 0 0 0 0 1];
            covmat_actual = agent_estimate_.getStateCovariancePositionVelocity();
            testCase.verifyEqual(covmat_actual, covmat_expected);
            covmat_sample2 = [...
                1 0 0 0 0 0;
                0 2 0 0 0 0;
                0 0 3 0 0 0;
                0 0 0 4 0 0;
                0 0 0 0 5 0;
                0 0 0 0 0 6];
            covmat_expected2 = covmat_sample2;
            % covmat_expected2 = zeros(6,6);
            agent_estimate_.setStateCovariancePositionVelocity(covmat_sample2);
            covmat_actual2 = agent_estimate_.getStateCovariancePositionVelocity();
            testCase.verifyEqual(covmat_actual2, covmat_expected2);
        end
    end
end