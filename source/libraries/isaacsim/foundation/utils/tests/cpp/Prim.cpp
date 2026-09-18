// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <doctest/doctest.h>
#include <isaacsim/foundation/objects/Prim.hpp>
#include <isaacsim/foundation/objects/Stage.hpp>
#include <isaacsim/foundation/utils/Prim.hpp>

#include <stdexcept>

using namespace isaacsim::foundation::utils;
namespace objects = isaacsim::foundation::objects;

TEST_SUITE("Prim")
{

    TEST_CASE("findMatchingPrimPaths")
    {
        objects::Stage stage = objects::Stage("openusd").createStage();

        // Build the prim hierarchy for the test case:
        //   / ()
        //   ├─ World ()
        //   │  ├─ A (Xform)
        //   │  ├─ A0 (Xform)
        //   │  │  ├─ B (Xform)
        //   │  │  ├─ B0 (Xform)
        //   │  │  │  ├─ C (Xform)
        //   │  │  ├─ B1 (Xform)
        //   │  │  │  ├─ C (Xform)
        //   │  │  ├─ B2 (Xform)
        //   │  │  │  ├─ C (Xform)
        //   │  ├─ A1 (Xform)
        //   │  │  ├─ B (Xform)
        //   │  │  ├─ B0 (Xform)
        //   │  │  │  ├─ C (Xform)
        //   │  │  ├─ B1 (Xform)
        //   │  │  │  ├─ C (Xform)
        //   │  │  ├─ B2 (Xform)
        //   │  │  │  ├─ C (Xform)
        stage.definePrim("/World/A");
        for (int i = 0; i < 2; ++i)
        {
            stage.definePrim("/World/A" + std::to_string(i));
            stage.definePrim("/World/A" + std::to_string(i) + "/B");
            for (int j = 0; j < 3; ++j)
            {
                stage.definePrim("/World/A" + std::to_string(i) + "/B" + std::to_string(j));
                stage.definePrim("/World/A" + std::to_string(i) + "/B" + std::to_string(j) + "/C");
            }
        }

        // Valid prim path
        CHECK_EQ(findMatchingPrimPaths("/World/A0/B0"), std::vector<std::string>{ "/World/A0/B0" });
        CHECK_EQ(findMatchingPrimPaths("/World/A0/B0", true),
                 (std::vector<std::string>{ "/World/A0/B0", "/World/A0/B0/C" }));
        // Regex
        // --
        CHECK_EQ(findMatchingPrimPaths("/World/A0/B[0-9]"),
                 (std::vector<std::string>{ "/World/A0/B0", "/World/A0/B1", "/World/A0/B2" }));
        CHECK_EQ(findMatchingPrimPaths("/World/A0/B[0-9]", true), (std::vector<std::string>{
                                                                      "/World/A0/B0",
                                                                      "/World/A0/B0/C",
                                                                      "/World/A0/B1",
                                                                      "/World/A0/B1/C",
                                                                      "/World/A0/B2",
                                                                      "/World/A0/B2/C",
                                                                  }));
        // --
        CHECK_EQ(findMatchingPrimPaths("/World/.*/.*[1,2]"),
                 (std::vector<std::string>{ "/World/A0/B1", "/World/A0/B2", "/World/A1/B1", "/World/A1/B2" }));
        CHECK_EQ(findMatchingPrimPaths("/World/.*/.*[1,2]", true), (std::vector<std::string>{
                                                                       "/World/A0/B1",
                                                                       "/World/A0/B1/C",
                                                                       "/World/A0/B2",
                                                                       "/World/A0/B2/C",
                                                                       "/World/A1/B1",
                                                                       "/World/A1/B1/C",
                                                                       "/World/A1/B2",
                                                                       "/World/A1/B2/C",
                                                                   }));
        // --
        CHECK_EQ(findMatchingPrimPaths(".*C.*"), std::vector<std::string>{});
        CHECK_EQ(findMatchingPrimPaths(".*C.*", true), (std::vector<std::string>{
                                                           "/World/A0/B0/C",
                                                           "/World/A0/B1/C",
                                                           "/World/A0/B2/C",
                                                           "/World/A1/B0/C",
                                                           "/World/A1/B1/C",
                                                           "/World/A1/B2/C",
                                                       }));

        REQUIRE_UNARY(stage.closeStage());
    }

    TEST_CASE("getAllMatchingChildPrims")
    {
        objects::Stage stage = objects::Stage("openusd").createStage();

        stage.definePrim("/World");
        stage.definePrim("/World/A0", "Sphere");
        for (int i = 0; i < 3; ++i)
        {
            stage.definePrim("/World/A0/B" + std::to_string(i), i % 2 ? "Cube" : "Sphere");
        }
        for (int i = 0; i < 3; ++i)
        {
            stage.definePrim("/World/A0/B0/C" + std::to_string(i), i % 2 ? "Cube" : "Sphere");
        }

        auto isSphere = [](const std::string& path) { return objects::Prim(path).getTypeName()[0] == "Sphere"; };

        // max_depth: nullopt (unlimited)
        CHECK_EQ(getAllMatchingChildPrims("/World/A0", isSphere),
                 (std::vector<std::string>{ "/World/A0/B0", "/World/A0/B2", "/World/A0/B0/C0", "/World/A0/B0/C2" }));
        // max_depth: 0
        CHECK_EQ(getAllMatchingChildPrims("/World/A0", isSphere, false, 0), std::vector<std::string>{});
        // max_depth: 1
        CHECK_EQ(getAllMatchingChildPrims("/World/A0", isSphere, false, 1),
                 (std::vector<std::string>{ "/World/A0/B0", "/World/A0/B2" }));
        // max_depth: 2
        CHECK_EQ(getAllMatchingChildPrims("/World/A0", isSphere, false, 2),
                 (std::vector<std::string>{ "/World/A0/B0", "/World/A0/B2", "/World/A0/B0/C0", "/World/A0/B0/C2" }));
        // include_self, max_depth: nullopt
        CHECK_EQ(getAllMatchingChildPrims("/World/A0", isSphere, true),
                 (std::vector<std::string>{ "/World/A0", "/World/A0/B0", "/World/A0/B2", "/World/A0/B0/C0",
                                            "/World/A0/B0/C2" }));
        // include_self, max_depth: 0
        CHECK_EQ(getAllMatchingChildPrims("/World/A0", isSphere, true, 0), (std::vector<std::string>{ "/World/A0" }));

        REQUIRE_UNARY(stage.closeStage());
    }

    TEST_CASE("getFirstMatchingChildPrim")
    {
        objects::Stage stage = objects::Stage("openusd").createStage();

        stage.definePrim("/World");
        stage.definePrim("/World/A");
        for (int i = 0; i < 5; ++i)
        {
            stage.definePrim("/World/A/B" + std::to_string(i), i % 2 ? "Cube" : "Sphere");
        }
        // Sphere is only reachable two levels below /World/D0.
        stage.definePrim("/World/D0", "Cube");
        stage.definePrim("/World/D0/E0", "Cube");
        stage.definePrim("/World/D0/E0/F0", "Sphere");

        auto isSphere = [](const std::string& path) { return objects::Prim(path).getTypeName()[0] == "Sphere"; };
        auto isXform = [](const std::string& path) { return objects::Prim(path).getTypeName()[0] == "Xform"; };
        auto neverMatch = [](const std::string&) { return false; };

        // first sphere under root (BFS)
        CHECK_EQ(getFirstMatchingChildPrim("/", isSphere, true), std::optional<std::string>{ "/World/A/B0" });
        // no match returns nullopt
        CHECK_EQ(getFirstMatchingChildPrim("/World/A", neverMatch), std::nullopt);
        // include_self: /World matches Xform
        CHECK_EQ(getFirstMatchingChildPrim("/World", isXform, true), std::optional<std::string>{ "/World" });
        // exclude_self: first Xform child of /World is /World/A
        CHECK_EQ(getFirstMatchingChildPrim("/World", isXform, false), std::optional<std::string>{ "/World/A" });

        // max_depth: nullopt (unlimited) - reaches the sphere two levels down
        CHECK_EQ(
            getFirstMatchingChildPrim("/World/D0", isSphere, false), std::optional<std::string>{ "/World/D0/E0/F0" });
        // max_depth: 0 - no descendants visited
        CHECK_EQ(getFirstMatchingChildPrim("/World/D0", isSphere, false, 0), std::nullopt);
        // max_depth: 1 - sphere is at depth 2, not reached
        CHECK_EQ(getFirstMatchingChildPrim("/World/D0", isSphere, false, 1), std::nullopt);
        // max_depth: 2 - sphere is reached
        CHECK_EQ(getFirstMatchingChildPrim("/World/D0", isSphere, false, 2),
                 std::optional<std::string>{ "/World/D0/E0/F0" });
        // include_self, max_depth: 0 - self is tested at depth 0 and matches
        CHECK_EQ(getFirstMatchingChildPrim("/World/D0/E0/F0", isSphere, true, 0),
                 std::optional<std::string>{ "/World/D0/E0/F0" });

        REQUIRE_UNARY(stage.closeStage());
    }

    TEST_CASE("getFirstMatchingParentPrim")
    {
        objects::Stage stage = objects::Stage("openusd").createStage();

        stage.definePrim("/World");
        stage.definePrim("/World/Cube", "Cube");
        stage.definePrim("/World/Cube/Sphere", "Sphere");

        auto isXform = [](const std::string& path) { return objects::Prim(path).getTypeName()[0] == "Xform"; };
        auto isSphere = [](const std::string& path) { return objects::Prim(path).getTypeName()[0] == "Sphere"; };
        auto neverMatch = [](const std::string&) { return false; };
        auto isRoot = [](const std::string& path) { return path == "/"; };

        // first Xform ancestor (exclude self)
        CHECK_EQ(getFirstMatchingParentPrim("/World/Cube/Sphere", isXform), std::optional<std::string>{ "/World" });
        // no match returns nullopt
        CHECK_EQ(getFirstMatchingParentPrim("/World/Cube/Sphere", neverMatch), std::nullopt);
        // pseudo-root "/" is not visited - returns nullopt
        CHECK_EQ(getFirstMatchingParentPrim("/World/Cube/Sphere", isRoot), std::nullopt);
        // include_self: /World/Cube/Sphere itself is Sphere
        CHECK_EQ(getFirstMatchingParentPrim("/World/Cube/Sphere", isSphere, true),
                 std::optional<std::string>{ "/World/Cube/Sphere" });
        // exclude_self: no other Sphere ancestor
        CHECK_EQ(getFirstMatchingParentPrim("/World/Cube/Sphere", isSphere, false), std::nullopt);

        REQUIRE_UNARY(stage.closeStage());
    }
}
