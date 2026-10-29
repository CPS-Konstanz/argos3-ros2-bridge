// Copyright 2026 The argos3-ros2-bridge Authors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/simulator/entities/light_entity.h>

#include <cstddef>
#include <iostream>
#include <string>
#include <vector>

#include "bridge/argos_ros_bridge.h"

using namespace argos;  // NOLINT(build/namespaces)

namespace
{
class CSwarmCenterOfMassTerminationLoopFunctions : public CLoopFunctions
{
public:
  CSwarmCenterOfMassTerminationLoopFunctions();

  void Init(TConfigurationNode & t_tree) override;
  void Reset() override;
  void PostStep() override;

private:
  struct SLightTarget
  {
    std::string Id;
    CVector3 Position;
  };

  void LoadTargetLightIds(TConfigurationNode & t_tree);
  void LoadTargetLightIdsFromNode(TConfigurationNode & t_tree);
  void LoadTargetsFromLights();
  static CEmbodiedEntity * GetEmbodiedEntity(CComposableEntity & composable);
  CVector3 ProjectForDistance(const CVector3 & position) const;
  bool TryGetSwarmCenterOfMass(CVector3 & center_of_mass) const;
  bool IsTargetLightId(const std::string & light_id) const;

  Real radius_;
  Real radius_squared_;
  bool enabled_;
  bool planar_distance_;
  bool terminated_;
  std::vector<std::string> target_light_ids_;
  std::vector<SLightTarget> targets_;
};

CSwarmCenterOfMassTerminationLoopFunctions::CSwarmCenterOfMassTerminationLoopFunctions()
: radius_(0.0f),
  radius_squared_(0.0f),
  enabled_(true),
  planar_distance_(true),
  terminated_(false)
{
}

void CSwarmCenterOfMassTerminationLoopFunctions::Init(TConfigurationNode & t_tree)
{
  GetNodeAttributeOrDefault(t_tree, "enabled", enabled_, enabled_);
  GetNodeAttributeOrDefault(t_tree, "radius", radius_, radius_);
  GetNodeAttributeOrDefault(t_tree, "planar_distance", planar_distance_, planar_distance_);

  LoadTargetLightIds(t_tree);
  LoadTargetsFromLights();
  terminated_ = false;

  if (!enabled_) {
    return;
  }

  if (radius_ <= 0.0f) {
    THROW_ARGOSEXCEPTION("The 'radius' attribute must be greater than zero");
  }
  if (targets_.empty()) {
    THROW_ARGOSEXCEPTION(
      "No target lights were found for "
      "'swarm_center_of_mass_termination_loop_functions'");
  }

  radius_squared_ = radius_ * radius_;
}

void CSwarmCenterOfMassTerminationLoopFunctions::Reset()
{
  terminated_ = false;
}

void CSwarmCenterOfMassTerminationLoopFunctions::PostStep()
{
  if (!enabled_ || terminated_) {
    return;
  }

  CVector3 center_of_mass;
  if (!TryGetSwarmCenterOfMass(center_of_mass)) {
    return;
  }

  const CVector3 projected_center_of_mass = ProjectForDistance(center_of_mass);
  for (std::size_t index = 0u; index < targets_.size(); ++index) {
    if (SquareDistance(
        projected_center_of_mass,
        ProjectForDistance(targets_[index].Position)) > radius_squared_)
    {
      continue;
    }

    terminated_ = true;
    std::cout << "Terminating simulation: swarm center of mass ("
              << center_of_mass.GetX() << ", "
              << center_of_mass.GetY() << ", "
              << center_of_mass.GetZ() << ") reached target "
              << '\'' << targets_[index].Id << "' at ("
              << targets_[index].Position.GetX() << ", "
              << targets_[index].Position.GetY() << ", "
              << targets_[index].Position.GetZ() << ") within radius "
              << radius_
              << std::endl;
    GetSimulator().Terminate();
    return;
  }
}

void CSwarmCenterOfMassTerminationLoopFunctions::LoadTargetLightIds(TConfigurationNode & t_tree)
{
  target_light_ids_.clear();
  LoadTargetLightIdsFromNode(t_tree);
  if (NodeExists(t_tree, "target_lights")) {
    TConfigurationNode & targets_node = GetNode(t_tree, "target_lights");
    if (!NodeAttributeExists(t_tree, "radius") &&
      NodeAttributeExists(targets_node, "radius"))
    {
      GetNodeAttribute(targets_node, "radius", radius_);
    }
    if (!NodeAttributeExists(t_tree, "planar_distance") &&
      NodeAttributeExists(targets_node, "planar_distance"))
    {
      GetNodeAttribute(targets_node, "planar_distance", planar_distance_);
    }
    LoadTargetLightIdsFromNode(targets_node);
  }
}

void CSwarmCenterOfMassTerminationLoopFunctions::LoadTargetLightIdsFromNode(
  TConfigurationNode & t_tree)
{
  TConfigurationNodeIterator target_light_it("target_light");
  for (target_light_it = target_light_it.begin(&t_tree);
    target_light_it != target_light_it.end();
    ++target_light_it)
  {
    std::string light_id;
    GetNodeAttribute(*target_light_it, "id", light_id);
    target_light_ids_.push_back(light_id);
  }
}

void CSwarmCenterOfMassTerminationLoopFunctions::LoadTargetsFromLights()
{
  targets_.clear();

  CSpace::TMapPerType & light_entities = GetSpace().GetEntitiesByType("light");
  for (CSpace::TMapPerType::iterator it = light_entities.begin();
    it != light_entities.end();
    ++it)
  {
    CLightEntity * light = any_cast<CLightEntity *>(it->second);
    if (light == nullptr) {
      continue;
    }
    if (!target_light_ids_.empty() && !IsTargetLightId(light->GetId())) {
      continue;
    }

    targets_.push_back({light->GetId(), light->GetPosition()});
  }
}

CEmbodiedEntity * CSwarmCenterOfMassTerminationLoopFunctions::GetEmbodiedEntity(
  CComposableEntity & composable)
{
  if (composable.HasComponent("body")) {
    return &composable.GetComponent<CEmbodiedEntity>("body");
  }
  if (composable.HasComponent("embodied")) {
    return &composable.GetComponent<CEmbodiedEntity>("embodied");
  }
  return nullptr;
}

CVector3 CSwarmCenterOfMassTerminationLoopFunctions::ProjectForDistance(const CVector3 & position)
const
{
  if (!planar_distance_) {
    return position;
  }
  return CVector3(position.GetX(), position.GetY(), 0.0f);
}

bool CSwarmCenterOfMassTerminationLoopFunctions::TryGetSwarmCenterOfMass(CVector3 & center_of_mass)
const
{
  CVector3 accumulated_position = CVector3::ZERO;
  std::size_t robot_count = 0u;

  CSpace & space = CSimulator::GetInstance().GetSpace();
  CEntity::TVector & entities = space.GetRootEntityVector();
  for (CEntity * entity : entities) {
    if (entity == nullptr) {
      continue;
    }

    CComposableEntity * composable = dynamic_cast<CComposableEntity *>(entity);
    if (composable == nullptr || !composable->HasComponent("controller")) {
      continue;
    }

    CControllableEntity & controllable =
      composable->GetComponent<CControllableEntity>("controller");
    if (dynamic_cast<ArgosRosBridge *>(&controllable.GetController()) == nullptr) {
      continue;
    }

    CEmbodiedEntity * embodied = GetEmbodiedEntity(*composable);
    if (embodied == nullptr) {
      continue;
    }

    accumulated_position += embodied->GetOriginAnchor().Position;
    ++robot_count;
  }

  if (robot_count == 0u) {
    return false;
  }

  center_of_mass = accumulated_position / static_cast<Real>(robot_count);
  return true;
}

bool CSwarmCenterOfMassTerminationLoopFunctions::IsTargetLightId(const std::string & light_id) const
{
  for (std::size_t index = 0u; index < target_light_ids_.size(); ++index) {
    if (target_light_ids_[index] == light_id) {
      return true;
    }
  }
  return false;
}

}  // namespace

REGISTER_LOOP_FUNCTIONS(
  CSwarmCenterOfMassTerminationLoopFunctions,
  "swarm_center_of_mass_termination_loop_functions")
