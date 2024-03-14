classdef global_fuser < handle & Utilities.utilities & Utilities.Assignment.assignment & GlobalFuser.utilities
    properties (GetAccess = public, SetAccess = private)
        assignmentDimension
        uncertainty
    end
    methods (Access = public)
        function this = global_fuser(scene, options)
            arguments
                scene (1, 1) trackingScenario
                options.Dimension {mustBeMember(options.Dimension, {'2D', 'SD'})} = "2D";
                options.Uncertainty (1, 1) logical = false;
            end
            this@Utilities.utilities(scene);
            this@Utilities.Assignment.assignment();
            this@GlobalFuser.utilities(scene);
            this.assignmentDimension = options.Dimension;
            this.uncertainty = options.Uncertainty;
        end
        function varargout = step(this, localTracks)
            arguments
                this (1, 1) GlobalFuser.global_fuser
                localTracks (:, 1) cell
            end
            if this.assignmentDimension == "2D"
                [cost, combinations] = this.generalized_likelihood_cost_2D(localTracks, ~this.uncertainty);
                [assignments, allTransAssn, transCombinations, seqCombinations] = this.assign_2D(cost, combinations);
                varargout = {assignments, allTransAssn, transCombinations, seqCombinations, cost};
            else
                cost = this.generalized_likelihood_cost_SD(localTracks, ~this.uncertainty);
                [assignments, achievedCost, exitFlag, output] = this.assign_SD(cost);
                varargout = {assignments, cost};
            end
        end
    end
    methods (Access = private)
        function [fullSeqAssn, allTransAssn, transCombinations, seqCombinations] = assign_2D(this, cost, combinations)
            arguments
                this (1, 1) GlobalFuser.global_fuser
                cost (:, 1) cell
                combinations (:, 2) {mustBeInteger, mustBePositive}
            end
            % Find the number of sensors from the number of combinations
            % dim^2 - dim - 2 * (combinationNum) = 0
            dim = roots([1, -1, -2 * numel(cost)]);
            dim = dim(dim > 0);

            seqList = this.list_sequential_order(dim);
            indices = ismember(combinations, seqList, "rows");
            seqCost = cost(indices);
            transitivityCost = cost(~indices);

            allSeqAssn = this.assign_2D_core_algorithm(seqCost);
            fullSeqAssn = this.format_assignments(allSeqAssn);
            if ~isempty(transitivityCost)
                allTransAssn = this.assign_2D_core_algorithm(transitivityCost);
                transCombinations = combinations(~indices, :);
            else
                allTransAssn = [];
                transCombinations = [];
            end
            seqCombinations = combinations(indices, :);
        end
        function [assignments, achievedCost, exitFlag, output] = assign_SD(this, cost)
            arguments
                this (1, 1) GlobalFuser.global_fuser
                cost {mustBeReal}
            end
            slvr = Utilities.Assignment.solver();
            [assignments, achievedCost, exitFlag, output] = slvr.milp_solver(cost, "HasExtraRowsAndColumns", this.uncertainty);
        end
        function allAssn = assign_2D_core_algorithm(this, cost)
            arguments
                this (1, 1) GlobalFuser.global_fuser
                cost (:, 1) cell
            end
            slvr = Utilities.Assignment.solver();
            for i = 1:numel(cost)
                if this.uncertainty
                    [assignments, achievedCost, exitFlag, output] = slvr.milp_solver(cost{i}, "HasExtraRowsAndColumns", true);
                else
                    [assignments, unassignedrows, unassignedcolumns] = assignmunkres(cost{i}, max(cost{i}, [], "all") + 1);
                end 
                allAssn{i, 1} = assignments;
            end
        end
    end
end