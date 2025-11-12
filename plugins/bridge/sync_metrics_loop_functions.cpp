#include "argos_ros_bridge.h"

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>

#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

using namespace argos;

namespace
{
class CSynchronizationMetricsLoopFunctions : public CLoopFunctions
{
public:
	CSynchronizationMetricsLoopFunctions();
	~CSynchronizationMetricsLoopFunctions() override;

	void Init(TConfigurationNode &t_tree) override;
	void Reset() override;
	void Destroy() override;
	void PostStep() override;

	void LogRobot(const std::string &robot_id,
				  ArgosRosBridge &controller,
				  const CVector3 &position);

	void OpenLog(bool truncate);
	void CloseLog();
	void WriteHeader();

private:
	std::ofstream log_stream_;
	std::string output_path_;
	bool write_header_;
	bool header_written_;
	bool enabled_;
	UInt32 flush_interval_;
	UInt32 steps_since_flush_;
	UInt64 step_counter_;
};

CSynchronizationMetricsLoopFunctions::CSynchronizationMetricsLoopFunctions() :
	output_path_("sync_metrics.csv"),
	write_header_(true),
	header_written_(false),
	enabled_(true),
	flush_interval_(100u),
	steps_since_flush_(0u),
	step_counter_(0u)
{
}

CSynchronizationMetricsLoopFunctions::~CSynchronizationMetricsLoopFunctions()
{
	CloseLog();
}

void CSynchronizationMetricsLoopFunctions::Init(TConfigurationNode &t_tree)
{
	GetNodeAttributeOrDefault(t_tree, "output", output_path_, output_path_);
	GetNodeAttributeOrDefault(t_tree, "enabled", enabled_, enabled_);
	GetNodeAttributeOrDefault(t_tree, "write_header", write_header_, write_header_);
	GetNodeAttributeOrDefault(t_tree, "flush_interval", flush_interval_, flush_interval_);

	OpenLog(true);
}

void CSynchronizationMetricsLoopFunctions::Reset()
{
	step_counter_ = 0u;
	steps_since_flush_ = 0u;
	OpenLog(true);
}

void CSynchronizationMetricsLoopFunctions::Destroy()
{
	CloseLog();
}

void CSynchronizationMetricsLoopFunctions::PostStep()
{
	if (!enabled_ || !log_stream_.is_open())
	{
		return;
	}

	++step_counter_;

	CSpace &space = CSimulator::GetInstance().GetSpace();
	CEntity::TVector &entities = space.GetRootEntityVector();
	for (CEntity *entity : entities)
	{
		if (entity == nullptr)
		{
			continue;
		}
		CComposableEntity *composable = dynamic_cast<CComposableEntity *>(entity);
		if (composable == nullptr || !composable->HasComponent("controller"))
		{
			continue;
		}
		CControllableEntity &controllable = composable->GetComponent<CControllableEntity>("controller");
		ArgosRosBridge *bridge = dynamic_cast<ArgosRosBridge *>(&controllable.GetController());
		if (bridge == nullptr)
		{
			continue;
		}
		CEmbodiedEntity *embodied = nullptr;
		if (composable->HasComponent("body"))
		{
			embodied = &composable->GetComponent<CEmbodiedEntity>("body");
		}
		else if (composable->HasComponent("embodied"))
		{
			embodied = &composable->GetComponent<CEmbodiedEntity>("embodied");
		}
		if (embodied == nullptr)
		{
			continue;
		}
		const CVector3 &pos = embodied->GetOriginAnchor().Position;
		LogRobot(entity->GetId(), *bridge, pos);
	}

	if (flush_interval_ > 0u)
	{
		++steps_since_flush_;
		if (steps_since_flush_ >= flush_interval_)
		{
			log_stream_.flush();
			steps_since_flush_ = 0u;
		}
	}
}

void CSynchronizationMetricsLoopFunctions::OpenLog(bool truncate)
{
	CloseLog();
	std::ios::openmode mode = std::ios::out;
	if (truncate)
	{
		mode |= std::ios::trunc;
	}
	else
	{
		mode |= std::ios::app;
	}
	log_stream_.open(output_path_, mode);
	if (!log_stream_.is_open())
	{
		THROW_ARGOSEXCEPTION("Unable to open synchronization metrics file '" << output_path_ << "'");
	}
	header_written_ = false;
}

void CSynchronizationMetricsLoopFunctions::CloseLog()
{
	if (log_stream_.is_open())
	{
		log_stream_.flush();
		log_stream_.close();
	}
}

void CSynchronizationMetricsLoopFunctions::WriteHeader()
{
	if (!write_header_ || header_written_)
	{
		return;
	}
	log_stream_ << "step"
			  << ",robot_id"
			  << ",lockstep"
			  << ",wait_ms"
			  << ",command_latency_steps"
			  << ",stale_command_ticks"
			  << ",current_step_index"
			  << ",last_command_step_index"
			  << ",pos_x"
			  << ",pos_y"
			  << ",pos_z"
			  << std::endl;
	header_written_ = true;
}

void CSynchronizationMetricsLoopFunctions::LogRobot(const std::string &robot_id,
								  ArgosRosBridge &controller,
								  const CVector3 &position)
{
	WriteHeader();
	std::ostringstream line;
	line << step_counter_
		 << ',' << robot_id
		 << ',' << (controller.IsLockstepControlEnabled() ? 1 : 0)
		 << ',' << std::fixed << std::setprecision(6) << controller.GetLastWaitDurationMs()
		 << ',' << std::defaultfloat << controller.GetLastCommandLatencySteps()
		 << ',' << controller.GetStaleCommandTicks()
		 << ',' << controller.GetCurrentStepIndex()
		 << ',' << controller.GetLastCommandStepIndex()
		 << ',' << std::fixed << std::setprecision(6) << position.GetX()
		 << ',' << position.GetY()
		 << ',' << position.GetZ();
	log_stream_ << line.str() << std::endl;
}

} // anonymous namespace

REGISTER_LOOP_FUNCTIONS(CSynchronizationMetricsLoopFunctions, "sync_metrics_loop_functions")
