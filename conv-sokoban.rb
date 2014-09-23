require 'matrix'
require 'pp'

str = [" GXX",
       "XRXX",
       "    ",
       "  B ",
       "  XX"]

m = Matrix.build(str[0].length, str.length) {|row, col| str[row][col] != 'X'}
pp m

#Find up predicates

(1..(str.length-1)).each do |y|
    (0..(str.length-1)).each do |x|
        if(m[x,y-1] && m[x,y])
            puts "(up (#{x},#{y-1}) (#{x},#{y}))"
        end
    end
end

