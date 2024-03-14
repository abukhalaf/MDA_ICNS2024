classdef utilities < handle & Utilities.utilities
    methods (Access = public)
        function this = utilities(scene)
            arguments
                scene (1, 1) trackingScenario
            end
            this@Utilities.utilities(scene);
        end
    end
    methods (Access = protected)
        function fullAssn = format_assignments(this, assignments)
            arguments
                this (1, 1) GlobalFuser.utilities
                assignments (:, 1) cell
            end
            dim = this.sensorHolderPlatformNumber;
            fullAssn = zeros(this.targetPlatformNumber, dim);
            firstAssnSz = size(assignments{1});
            fullAssn(1:firstAssnSz(1), 1:firstAssnSz(2)) = assignments{1};
            for i = 2:(dim - 1)
                fullAssn = sortrows(fullAssn, i, "descend");
                assignments{i} = sortrows(assignments{i}, 1, "descend");

                assn = fullAssn(:, i);
                assn = assn(assn > 0);
                [ind1, ind2] = ismember(assignments{i}(:, 1), assn);
                fullAssn(ind2(ind2 > 0), i:(i + 1)) = assignments{i}(ind1, :);
            end
            fullAssn = sortrows(fullAssn, 1);
            ind = ismember(fullAssn, zeros(1, dim), "rows");
            fullAssn(ind, :) = [];
        end
    end
    methods (Access = protected, Static = true)
        function fullCost = format_cost(cost)
            arguments
                cost {mustBeReal}
            end
            sz = size(cost);
            szFullCost = sz + 1;
            ind = cell(numel(szFullCost), 1);
            for i = 1:numel(szFullCost)
                ind{i} = 2:(szFullCost(i));
            end
            fullCost = ones(szFullCost) * sum(cost, "all") * 1e6;
            fullCost(ind{:}) = cost;
            fullCost(1) = 0;
        end
        function list = list_sequential_order(dim)
            arguments
                dim {mustBeInteger, mustBePositive}
            end
            list = repmat([1, 2], (dim - 1), 1) + (0:(dim - 2))';
        end
    end
end