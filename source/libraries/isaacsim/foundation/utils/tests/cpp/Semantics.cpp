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
#include <isaacsim/foundation/objects/Stage.hpp>
#include <isaacsim/foundation/utils/Semantics.hpp>

using namespace isaacsim::foundation::utils;
namespace objects = isaacsim::foundation::objects;

TEST_SUITE("Semantics")
{

    TEST_CASE("addLabels")
    {
        objects::Stage stage = objects::Stage("openusd").createStage();

        stage.definePrim("/World/A", "Cube");
        CHECK_EQ(getLabels("/World/A"), (std::unordered_map<std::string, std::vector<std::string>>{}));

        // Add labels to default and custom taxonomy
        addLabels("/World/A", std::vector<std::string>{ "label_0", "label_1" });
        addLabels("/World/A", std::vector<std::string>{ "label_1", "label_2", "label_3" }, "test");
        CHECK_EQ(getLabels("/World/A"), (std::unordered_map<std::string, std::vector<std::string>>{
                                            { "class", { "label_0", "label_1" } },
                                            { "test", { "label_1", "label_2", "label_3" } },
                                        }));

        // Add a new label to existing ones
        addLabels("/World/A", std::string{ "label_4" });
        addLabels("/World/A", std::string{ "label_4" }); // Add same label again (no-op)
        CHECK_EQ(getLabels("/World/A"), (std::unordered_map<std::string, std::vector<std::string>>{
                                            { "class", { "label_0", "label_1", "label_4" } },
                                            { "test", { "label_1", "label_2", "label_3" } },
                                        }));

        REQUIRE_UNARY(stage.closeStage());
    }

    TEST_CASE("getLabels")
    {
        objects::Stage stage = objects::Stage("openusd").createStage();

        stage.definePrim("/World/A", "Cube");
        stage.definePrim("/World/B", "Xform");
        CHECK_EQ(getLabels("/World/A"), (std::unordered_map<std::string, std::vector<std::string>>{}));
        CHECK_EQ(getLabels("/World/B"), (std::unordered_map<std::string, std::vector<std::string>>{}));

        // Add labels
        addLabels("/World/A", std::vector<std::string>{ "label_0", "label_1" });
        addLabels("/World/B", std::vector<std::string>{ "label_1", "label_2", "label_3" }, "test");

        // Get labels from specific prims
        CHECK_EQ(getLabels("/World/A"),
                 (std::unordered_map<std::string, std::vector<std::string>>{ { "class", { "label_0", "label_1" } } }));
        CHECK_EQ(getLabels("/World/B"), (std::unordered_map<std::string, std::vector<std::string>>{
                                            { "test", { "label_1", "label_2", "label_3" } } }));

        // Get labels from prim without semantics applied
        CHECK_EQ(getLabels("/World"), (std::unordered_map<std::string, std::vector<std::string>>{}));

        // Get labels from prim without direct semantics but with labeled descendants
        CHECK_EQ(getLabels("/World", /*includeDescendants=*/true),
                 (std::unordered_map<std::string, std::vector<std::string>>{
                     { "class", { "label_0", "label_1" } },
                     { "test", { "label_1", "label_2", "label_3" } },
                 }));

        REQUIRE_UNARY(stage.closeStage());
    }

    TEST_CASE("removeLabels")
    {
        objects::Stage stage = objects::Stage("openusd").createStage();

        stage.definePrim("/World/A", "Cube");
        CHECK_EQ(getLabels("/World/A"), (std::unordered_map<std::string, std::vector<std::string>>{}));

        // Add labels
        addLabels("/World/A", std::vector<std::string>{ "label_0", "label_1", "label_4" });
        addLabels("/World/A", std::vector<std::string>{ "label_1", "label_2", "label_3" }, "test");
        CHECK_EQ(getLabels("/World/A"), (std::unordered_map<std::string, std::vector<std::string>>{
                                            { "class", { "label_0", "label_1", "label_4" } },
                                            { "test", { "label_1", "label_2", "label_3" } },
                                        }));

        // Remove from a specific taxonomy
        removeLabels("/World/A", std::string{ "label_2" }, std::string{ "test" });
        CHECK_EQ(getLabels("/World/A"), (std::unordered_map<std::string, std::vector<std::string>>{
                                            { "class", { "label_0", "label_1", "label_4" } },
                                            { "test", { "label_1", "label_3" } },
                                        }));

        // Remove from all taxonomies
        removeLabels("/World/A", std::string{ "label_1" });
        CHECK_EQ(getLabels("/World/A"), (std::unordered_map<std::string, std::vector<std::string>>{
                                            { "class", { "label_0", "label_4" } },
                                            { "test", { "label_3" } },
                                        }));

        // Remove from descendants
        removeLabels("/World", std::string{ "label_4" }, std::nullopt, /*includeDescendants=*/true);
        CHECK_EQ(getLabels("/World/A"), (std::unordered_map<std::string, std::vector<std::string>>{
                                            { "class", { "label_0" } },
                                            { "test", { "label_3" } },
                                        }));

        // Removing a non-existent label is a no-op
        removeLabels("/World/A", std::string{ "label_5" });
        CHECK_EQ(getLabels("/World/A"), (std::unordered_map<std::string, std::vector<std::string>>{
                                            { "class", { "label_0" } },
                                            { "test", { "label_3" } },
                                        }));

        REQUIRE_UNARY(stage.closeStage());
    }

    TEST_CASE("removeAllLabels")
    {
        objects::Stage stage = objects::Stage("openusd").createStage();

        stage.definePrim("/World/A", "Cube");
        CHECK_EQ(getLabels("/World/A"), (std::unordered_map<std::string, std::vector<std::string>>{}));

        // Add labels
        addLabels("/World/A", std::vector<std::string>{ "label_0", "label_1", "label_4" });
        addLabels("/World/A", std::vector<std::string>{ "label_1", "label_2", "label_3" }, "test");
        CHECK_EQ(getLabels("/World/A"), (std::unordered_map<std::string, std::vector<std::string>>{
                                            { "class", { "label_0", "label_1", "label_4" } },
                                            { "test", { "label_1", "label_2", "label_3" } },
                                        }));

        // Remove all labels but keep taxonomies
        removeAllLabels("/World/A");
        CHECK_EQ(getLabels("/World/A"), (std::unordered_map<std::string, std::vector<std::string>>{
                                            { "class", {} },
                                            { "test", {} },
                                        }));

        // Remove taxonomies on the parent only (child /World/A is unaffected)
        removeAllLabels("/World", /*removeTaxonomies=*/true);
        CHECK_EQ(getLabels("/World/A"), (std::unordered_map<std::string, std::vector<std::string>>{
                                            { "class", {} },
                                            { "test", {} },
                                        }));

        // Remove taxonomies on parent and all descendants
        removeAllLabels("/World", /*removeTaxonomies=*/true, /*includeDescendants=*/true);
        CHECK_EQ(getLabels("/World/A"), (std::unordered_map<std::string, std::vector<std::string>>{}));

        REQUIRE_UNARY(stage.closeStage());
    }
}
