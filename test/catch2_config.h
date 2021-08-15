/**
 * @file catch2_config.h
 * @author Marcel Flottmann
 * @author Julian Gaal
 *
 * This file
 * * enables CATCH_CONFIG_FAST_COMPILE
 * * offers REQUIRE_EACH, a helper function that does a Catch2 REQUIRE on each
 * element of a container
 */

#define CATCH_CONFIG_FAST_COMPILE

#include <catch2/catch.hpp>
#include <algorithm>

template<class C, typename D>
void REQUIRE_EACH(const C& obj, D res)
{
    using elem_type = typename C::value_type;
    std::for_each(obj.cbegin(), obj.cend(), [&res](const elem_type & elem)
    {
        REQUIRE(elem == res);
    });
}
