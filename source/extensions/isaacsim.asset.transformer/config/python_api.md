# Public API for module isaacsim.asset.transformer:

## Classes

- class RuleInterface(ABC)
  - def __init__(self, source_stage: Usd.Stage, package_root: str, destination_path: str, args: dict[str, Any])
  - def process_rule(self) -> str | None
  - def log_operation(self, message: str)
  - def get_operation_log(self) -> list[str]
  - def add_affected_stage(self, stage_identifier: str)
  - def get_affected_stages(self) -> list[str]
  - def get_configuration_parameters(self) -> list[RuleConfigurationParam]

- class RuleSpec
  - name: str
  - type: str
  - destination: str | None
  - params: dict[str, Any]
  - enabled: bool
  - def to_dict(self) -> dict[str, Any]
  - static def from_dict(data: dict[str, Any]) -> RuleSpec

- class RuleProfile
  - profile_name: str
  - version: str | None
  - rules: list[RuleSpec]
  - interface_asset_name: str | None
  - output_package_root: str | None
  - flatten_source: bool
  - base_name: str | None
  - def to_dict(self) -> dict[str, Any]
  - def to_json(self) -> str
  - static def from_dict(data: dict[str, Any]) -> RuleProfile
  - static def from_json(json_str: str) -> RuleProfile

- class RuleExecutionResult
  - rule: RuleSpec
  - success: bool
  - log: list[dict[str, Any]]
  - affected_stages: list[str]
  - error: str | None
  - started_at: str
  - finished_at: str | None
  - def close(self)

- class ExecutionReport
  - profile: RuleProfile
  - input_stage_path: str
  - package_root: str
  - started_at: str
  - finished_at: str | None
  - results: list[RuleExecutionResult]
  - output_stage_path: str | None
  - def to_dict(self) -> dict[str, Any]
  - def to_json(self) -> str
  - def close(self)

- class RuleRegistry
  - def __init__(self)
  - def register(self, rule_cls: type[RuleInterface])
  - def get(self, rule_type: str) -> type[RuleInterface] | None
  - def clear(self)
  - def list_rules(self) -> dict[str, type[RuleInterface]]
  - def list_rule_types(self) -> list[str]

- class AssetTransformerManager
  - def __init__(self, registry: RuleRegistry | None = None)
  - [property] def registry(self) -> RuleRegistry
  - def run(self, input_stage_path: str, profile: RuleProfile, package_root: str | None = None) -> ExecutionReport

- class RuleConfigurationParam
  - name: str
  - display_name: str
  - param_type: type
  - description: str | None
  - default_value: object | None
