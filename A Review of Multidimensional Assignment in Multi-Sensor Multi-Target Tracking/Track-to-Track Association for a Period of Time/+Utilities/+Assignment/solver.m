classdef solver < handle
    methods (Access = public)
        function this = solver()
        end
        function [assignments, achievedCost, exitFlag, output] = milp_solver(this, cost, options)
            arguments
                this (1, 1) Utilities.Assignment.solver
                cost {mustBeReal}
                options.HasExtraRowsAndColumns (1, 1) logical = false;
            end
            % Identify Problem Specific Numbers
            [sz, dim, elementNum, constraintNum] = this.get_numbers(cost, options.HasExtraRowsAndColumns);

            % Identify intlinprog parameters
            f = reshape(cost, [elementNum, 1]);
            intcon = 1:elementNum;
            A = [];
            b = [];
            Aeq = zeros(constraintNum, elementNum);
            beq = ones(constraintNum, 1);
            lb = zeros(elementNum, 1);
            ub = ones(elementNum, 1);
            x0 = [];
            opt = optimoptions("intlinprog", "Display", "off", ...
                               "RootLPAlgorithm", "primal-simplex");

            % Modify Initialized Aeq
            constraintCounter = 1;
            hypotheses = this.get_hypotheses(sz) - double(options.HasExtraRowsAndColumns);
            for i = 1:dim
                hypo = hypotheses(:, i);
                for j = 1:max(hypo)
                    ind = (hypo == j);
                    Aeq(constraintCounter, ind) = 1;
                    constraintCounter = constraintCounter + 1;
                end
            end
            
            % Solve the problem
            [assignments, achievedCost, exitFlag, output] = intlinprog(f, intcon, A, b, Aeq, beq, lb, ub, x0, opt);
            assn = cell(dim, 1);
            [assn{:}] = ind2sub(sz, find(assignments == 1));
            assignments = horzcat(assn{:});
            if options.HasExtraRowsAndColumns
                assignments = assignments - 1;
            end
        end
    end
    methods (Access = private, Static = true)
        function [sz, dimension, elementNum, constraintNum] = get_numbers(cost, hasExtraRowsAndColumns)
            arguments
                cost {mustBeReal}
                hasExtraRowsAndColumns (1, 1) logical
            end
            sz = size(cost);
            dimension = numel(sz);
            elementNum = numel(cost);
            constraintNum = sum(sz) - dimension * double(hasExtraRowsAndColumns);
        end
        function hypotheses = get_hypotheses(sz)
            arguments
                sz (1, :) {mustBeInteger, mustBePositive}
            end
            cost = zeros(sz);
            dim = numel(sz);
            subIndices = cell(dim, 1);
            [subIndices{:}] = ind2sub(sz, 1:numel(cost));
            hypotheses = vertcat(subIndices{:})';
        end
    end
end