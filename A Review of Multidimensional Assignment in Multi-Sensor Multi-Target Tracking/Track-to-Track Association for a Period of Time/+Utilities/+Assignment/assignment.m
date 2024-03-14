% assignment class is an implementation of the equations described in [1].
%
% [1] L. M. Kaplan, Y. Bar-Shalom and W. D. Blair, "Assignment costs for
%     multiple sensor track-to-track association," in IEEE Transactions on
%     Aerospace and Electronic Systems, vol. 44, no. 2, pp. 655-677, April
%     2008, doi: 10.1109/TAES.2008.4560213.
classdef assignment < handle
    methods (Access = public)
        function this = assignment()
        end
        function [cost, combinations] = generalized_likelihood_cost_2D(this, localTracks, isTrivial)
            arguments
                this (1, 1) Utilities.Assignment.assignment
                localTracks (:, 1) cell
                isTrivial (1, 1) logical = false;
            end
            dim = numel(localTracks);
            combinations = sortrows(nchoosek(1:dim, 2), 1);
            cost = cell(size(combinations, 1), 1);
            for i = 1:size(combinations, 1)
                trackNum = [numel(localTracks{combinations(i, 1)}), numel(localTracks{combinations(i, 2)})];
                if isTrivial
                    cost{i} = Inf(trackNum);
                    startInd = 1;
                else
                    trackNum = trackNum + 1;
                    cost{i} = Inf(trackNum);
                    cost{i}(1) = 0;
                    startInd = 2;
                end
                subIndices = cell(2, 1);
                [subIndices{:}] = ind2sub(size(cost{i}), 1:numel(cost{i}));
                hypotheses = vertcat(subIndices{:})';
                for j = startInd:size(hypotheses, 1)
                    tupleArr = hypotheses(j, :) - double(~isTrivial);
                    tuple = num2cell(hypotheses(j, :));
                    generalized_neg_loglikelihood = this.generalized_neg_loglikelihood(localTracks(combinations(i, :)), tupleArr);
                    if ~isTrivial
                        dummies = find(tupleArr == 0);
                        if ~isempty(dummies)
                            coeff = 0;
                            for k = 1:numel(dummies)
                                coeff = coeff + numel(localTracks{dummies(k)});
                            end
                            generalized_neg_loglikelihood = generalized_neg_loglikelihood * coeff;
                        end
                    end
                    cost{i}(tuple{:}) = generalized_neg_loglikelihood;
                end
            end
        end
        function cost = generalized_likelihood_cost_SD(this, localTracks, isTrivial)
            arguments
                this (1, 1) Utilities.Assignment.assignment
                localTracks (:, 1) cell
                isTrivial (1, 1) logical = false;
            end
            dim = numel(localTracks);
            trackNum = zeros(1, dim);
            for i = 1:dim
                trackNum(i) = numel(localTracks{i});
            end
            if isTrivial
                cost = Inf(trackNum);
                startInd = 1;
            else
                trackNum = trackNum + 1;
                cost = Inf(trackNum);
                cost(1) = 0;
                startInd = 2;
            end
            subIndices = cell(dim, 1);
            [subIndices{:}] = ind2sub(size(cost), 1:numel(cost));
            hypotheses = vertcat(subIndices{:})';
            for i = startInd:size(hypotheses, 1)
                tupleArr = hypotheses(i, :) - double(~isTrivial);
                tuple = num2cell(hypotheses(i, :));
                generalized_neg_loglikelihood = this.generalized_neg_loglikelihood(localTracks, tupleArr);
                if ~isTrivial
                    dummies = find(tupleArr == 0);
                    if ~isempty(dummies)
                        coeff = 0;
                        for k = 1:numel(dummies)
                            coeff = coeff + numel(localTracks{dummies(k)});
                        end
                        generalized_neg_loglikelihood = generalized_neg_loglikelihood * coeff;
                    end
                end
                cost(tuple{:}) = generalized_neg_loglikelihood;
            end
        end
    end
    methods (Access = private, Static = true)
        function generalized_likelihood = generalized_neg_loglikelihood(localTracks, hypothesis)
            arguments
                localTracks (:, 1) cell
                hypothesis (1, :) {mustBeInteger, mustBeNonnegative}
            end
            ind = find(hypothesis ~= 0);
            tempCost = 0;
            Rfn = zeros(6, 6);
            xfn = zeros(6, 1);
            for k = 1:numel(ind)
                trackPair1 = localTracks{ind(k)}(hypothesis(ind(k)));
                Ri = trackPair1.StateCovariance;
                Rfn = Rfn + inv(Ri);
            end
            Rfn = inv(Rfn);
            for k = 1:numel(ind)
                trackPair1 = localTracks{ind(k)}(hypothesis(ind(k)));
                xi = trackPair1.State;
                Ri = trackPair1.StateCovariance; 
                xfn = xfn + inv(Ri) * xi;
            end
            xfn = Rfn * xfn;
            for k = 1:numel(ind)
                trackPair1 = localTracks{ind(k)}(hypothesis(ind(k)));
                tempCost = tempCost + log(1 / (sqrt(det(2 * pi * trackPair1.StateCovariance)))) - (0.5 * ((trackPair1.State - xfn)') * inv(trackPair1.StateCovariance) * (trackPair1.State - xfn));
            end
            generalized_likelihood = -tempCost;
        end
    end
end