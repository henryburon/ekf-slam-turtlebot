#include <catch2/catch_test_macros.hpp>
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/svg.hpp"
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <sstream>
#include <fstream>

TEST_CASE("Svg::DrawVector generates correct SVG output", "[Svg]")
{
    SECTION("Test Point")
    {
        // File path
        std::string filepath = "../tmp/test_frames.svg";
        
        std::ofstream svg_file(filepath);
        turtlelib::Svg mySvg;
 
        mySvg.DrawPoint(0, 0, "red", svg_file);

        std::string expected_first_line = "<circle cx=\"408\" cy=\"528\" r=\"3\" stroke=\"red\" fill=\"red\" stroke-width=\"1\"/>\n";
        REQUIRE(mySvg.getSvgString() == expected_first_line);
    }

    

        std::string expected_first_line_vector = "<line x1=\"456\" x2=\"360\" y1=\"624\" y2=\"336\" stroke=\"blue\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />";
        
        std::string expected_line1 = "<g>";
        std::string expected_line2 = "<line x1=\"408\" x2=\"312\" y1=\"240\" y2=\"240\" stroke=\"red\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />";
        std::string expected_line3 = "<line x1=\"312\" x2=\"312\" y1=\"144\" y2=\"240\" stroke=\"green\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />";
        std::string expected_line4 = "<text x=\"312\" y=\"240\">{T_ab}</text>";
        std::string expected_line5 = "</g>";

}
