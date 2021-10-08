classdef Ees
    
    properties
        E = interval(0);
        e = interval(0);
    end
    
    methods
        function obj = Ees(E_inp,e_inp)
            obj.E = E_inp;
            obj.e = e_inp;
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
            
            output = Ees(inp1.E * inp2.E, inp1.e * inp2.e + inp1.E * inp2.e + inp1.e * inp2.E);
        end
    end
end

