classdef analysis < handle & Utilities.utilities
    methods (Access = public)
        function this = analysis(scene)
            arguments
                scene (1, 1) trackingScenario
            end
            this@Utilities.utilities(scene);
        end
        function [trueAssnNum, faultyAssnNum, transitivityErrorNum, trueAssignments] = count_true_assignments_2D(this, assignments, transitivityAssignments, localTracks)
            arguments
                this (1, 1) Utilities.analysis
                assignments (:, 1) cell
                transitivityAssignments (:, 2) cell
                localTracks cell
            end
            trueAssnNum = 0;
            faultyAssnNum = 0;
            transitivityErrorNum = 0;
            trueAssignments = zeros(this.totalTimeStep, 1);
            for i = 1:this.totalTimeStep
                assn = assignments{i};
                transAssn = transitivityAssignments{i, 1};
                transCom = transitivityAssignments{i, 2};
                lclTrcks = localTracks(i, :);

                transTest = this.transitivity_test(assn, transAssn, transCom);
                assnTest = this.assignment_test(assn, lclTrcks);
                if ~transTest
                    transitivityErrorNum = transitivityErrorNum + 1;
                end
                if ~assnTest
                    faultyAssnNum = faultyAssnNum + 1;
                end
                if transTest && assnTest
                    trueAssignments(i) = 1;
                    trueAssnNum = trueAssnNum + 1;
                end
            end
        end
        function [trueAssnNum, faultyAssnNum, trueAssignments] = count_true_assignments_SD(this, assignments, localTracks)
            arguments
                this (1, 1) Utilities.analysis
                assignments (:, 1) cell
                localTracks cell
            end
            trueAssnNum = 0;
            faultyAssnNum = 0;
            trueAssignments = zeros(this.totalTimeStep, 1);
            for i = 1:this.totalTimeStep
                assn = assignments{i};
                lclTrcks = localTracks(i, :);
                assnTest = this.assignment_test(assn, lclTrcks);
                if assnTest
                    trueAssignments(i) = 1;
                    trueAssnNum = trueAssnNum + 1;
                else
                    faultyAssnNum = faultyAssnNum + 1;
                end
            end
        end
    end
    methods (Access = private, Static = true)
        function successFlag = assignment_test(assignment, localTrack)
            arguments
                assignment {mustBeInteger, mustBeNonnegative}
                localTrack (1, :) cell
            end
            successFlag = false;
            for i = 1:size(assignment, 1)
                assn = assignment(i, :);
                tgtInd = [];
                for j = 1:numel(assn)
                    if assn(j)
                        tgtInd(1, end + 1) = localTrack{j}(assn(j)).ObjectAttributes.TargetIndex;
                    end
                end
                if all(tgtInd == tgtInd(1))
                    successFlag = true;
                else
                    successFlag = false;
                    break;
                end
            end
        end
        function successFlag = transitivity_test(assignment, transitivityAssignment, transitiviyCombination)
            arguments
                assignment {mustBeInteger, mustBeNonnegative}
                transitivityAssignment 
                transitiviyCombination
            end
            successFlag = false;
            if isempty(transitivityAssignment)
                successFlag = true;
                return;
            end
            for i = 1:size(transitiviyCombination, 1)
                comb = transitiviyCombination(i, :);
                assn = sortrows(assignment(:, comb), 1);
                transitivityAssignment{i} = sortrows(transitivityAssignment{i}, 1);
                ind = ismember(assn, zeros(1, 2), "rows");
                assn(ind, :) = [];
                szAssn = size(assn);
                szTransAssn = size(transitivityAssignment{i});
                if any(szAssn ~= szTransAssn, "all") || any(assn ~= transitivityAssignment{i}, "all")
                    successFlag = false;
                    break;
                else
                    successFlag = true;
                end

            end
        end
    end
end