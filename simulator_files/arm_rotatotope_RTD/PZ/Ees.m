classdef Ees
    properties
        E = interval(0);
        e = interval(0);
    end
    
    methods
        function obj = Ees(E_inp,e_inp)
            if nargin == 2
                if isa(E_inp, 'interval')
                    obj.E = E_inp;
                else
                    obj.E = interval(E_inp);
                end
                if isa(e_inp, 'interval')
                    obj.e = e_inp;
                else
                    obj.e = interval(e_inp);
                end
            end
        end
        
        function output = plus(inp1, inp2)
            if isa(inp1, 'double')
                output = inp2;
                output.E = output.E + inp1;
                return;
            end
            
            if isa(inp2, 'double')
                output = inp1;
                output.E = output.E + inp2;
                return;
            end
            
            if isa(inp1, 'interval')
                output = inp2;
                output.E = output.E + inp1;
                output.e = output.e + inp1;
                return;
            end
            
            if isa(inp2, 'interval')
                output = inp1;
                output.E = output.E + inp2;
                output.e = output.e + inp2;
                return;
            end
            
            output = Ees(inp1.E + inp2.E, inp1.e + inp2.e);
        end
        
        function output = mtimes(inp1, inp2)
            if isa(inp1, 'double')
                output = Ees(inp1 * inp2.E, inp1 * inp2.e);
                return;
            end
            
            if isa(inp2, 'double')
                output = Ees(inp1.E * inp2, inp1.e * inp2);
                return;
            end
            
            if isa(inp1, 'interval')
                range = inp1 * (inp2.E + inp2.e);
                output = Ees(0 * range, range);
                return;
            end
            
            if isa(inp2, 'interval')
                range = (inp1.E + inp1.e) * inp2;
                output = Ees(0 * range, range);
                return;
            end
            
            output = Ees(inp1.E * inp2.E, inp1.e * inp2.e + inp1.E * inp2.e + inp1.e * inp2.E);
        end
        
        function output = times(inp1, inp2)
            if isa(inp1, 'double')
                output = Ees(inp1 .* inp2.E, inp1 .* inp2.e);
                return;
            end
            
            if isa(inp2, 'double')
                output = Ees(inp1.E .* inp2, inp1.e .* inp2);
                return;
            end
            
            if isa(inp1, 'interval')
                output = Ees(0, inp1 .* (inp2.E + inp2.e));
                return;
            end
            
            if isa(inp2, 'interval')
                output = Ees(0, (inp1.E + inp1.e) .* inp2);
                return;
            end
            
            output = Ees(inp1.E .* inp2.E, inp1.e .* inp2.e + inp1.E .* inp2.e + inp1.e .* inp2.E);
        end
        
        function output = uminus(obj)
            output = Ees(-obj.E, obj.e);
        end
        
        function output = ctranspose(obj)
            output = Ees(obj.E', obj.e');
        end
        
        function output = transpose(obj)
            output = Ees(obj.E.', obj.e.');
        end
        
        function newObj = subsref(obj,S)
            if length(S) == 1 && strcmp(S.type,'()')
                %obtain sub-intervals from the interval object
                newObj = obj;
                % only one index specified
                if length(S.subs)==1
                    newObj.E=obj.E(S.subs{1});
                    newObj.e=obj.e(S.subs{1});
                %two indices specified
                elseif length(S.subs)==2
                    %Select column of obj
                    if strcmp(S.subs{1},':')
                        column=S.subs{2};
                        newObj.E=obj.E(:,column);
                        newObj.e=obj.e(:,column);
                    %Select row of V    
                    elseif strcmp(S.subs{2},':')
                        row=S.subs{1};
                        newObj.E=obj.E(row,:);
                        newObj.e=obj.e(row,:);
                    %Select single element of V    
                    elseif isnumeric(S.subs{1}) && isnumeric(S.subs{1})
                        row=S.subs{1};
                        column=S.subs{2};
                        newObj.E=obj.E(row,column);
                        newObj.e=obj.e(row,column);
                    end
                end
            else
                % call build in subsref function as a default
                newObj = builtin('subsref', obj, S);
            end
        end
    end
end

