# NLP Interface for Robot Navigation

This module provides a natural language interface for controlling the Flembot robot. It translates human-readable commands into the robot's native navigation command format.

## Overview

The NLP interface uses a hybrid approach:
1. **Local Pattern Matching**: Fast, offline translation for common commands
2. **LLM Translation**: API-based translation for complex or ambiguous commands

## Installation

The NLP interface is a standalone Python module. It requires Python 3.8+ and has no external dependencies for basic local pattern matching.

For LLM-based translation, you need an API key from one of the supported providers:
- OpenAI (default)
- Anthropic (Claude)
- Azure OpenAI

### Setting up API Keys

```bash
# For OpenAI
export OPENAI_API_KEY="your-api-key-here"

# For Anthropic
export ANTHROPIC_API_KEY="your-api-key-here"

# For Azure OpenAI
export AZURE_OPENAI_API_KEY="your-api-key-here"
export AZURE_OPENAI_ENDPOINT="https://your-endpoint.openai.azure.com"
```

## Usage

### Standalone Python Usage

```python
from nlp_interface import NLPCommandTranslator

# Create translator (local patterns only, no API needed)
translator = NLPCommandTranslator(use_llm=False)

# Or with LLM support
translator = NLPCommandTranslator(use_llm=True, llm_provider="openai")

# Translate commands
command, method = translator.translate("move forward two meters")
print(command)  # Output: forward:2.0
print(method)   # Output: local

command, method = translator.translate("go towards the wall and then turn around")
print(command)  # Output: forward_until_close:front:0.5;turn_left:180
print(method)   # Output: llm
```

### Running the Demo

```bash
# Test local pattern matching
python -m nlp_interface.translator

# Test LLM translation (requires API key)
python -m nlp_interface.llm_client
```

### ROS2 Integration

The NLP interface includes a ROS2 node for integration with the reactive navigation controller.

```bash
# Start the reactive navigation controller
ros2 run gazebo_differential_drive_robot reactive_navigation_controller.py

# Start the NLP navigation node (from the nlp_interface directory)
python nlp_navigation_node.py

# Send natural language commands
ros2 topic pub /nlp/command std_msgs/msg/String "data: 'move forward two meters'" --once

# Monitor translated commands
ros2 topic echo /navigation/command

# Monitor translation feedback
ros2 topic echo /nlp/feedback
```

## Supported Commands

### Local Pattern Matching (No API Required)

These commands are translated locally without requiring an LLM API:

| Natural Language | Robot Command |
|-----------------|---------------|
| "stop" / "halt" | `stop` |
| "move forward" / "go straight" | `forward` |
| "move forward 2 meters" | `forward:2.0` |
| "forward two meters" | `forward:2.0` |
| "move backwards 1 meter" | `backward:1.0` |
| "turn left" | `turn_left` |
| "turn left 45 degrees" | `turn_left:45` |
| "turn right ninety degrees" | `turn_right:90` |
| "turn around" / "u-turn" | `turn_left:180` |
| "follow the left wall" | `follow_wall:left:0.5` |
| "follow right wall at 0.3 meters" | `follow_wall:right:0.3` |

### LLM Translation (API Required)

Complex commands that require context understanding are handled by the LLM:

| Natural Language | Robot Command |
|-----------------|---------------|
| "move forward until you reach a wall then turn left" | `forward_until_close:front:0.5:turn_left` |
| "navigate around the obstacle" | `forward_until_close:front:0.5:turn_left;forward:1.0;turn_right:90;forward:2.0` |
| "scan the room" | `turn_left:90;turn_left:90;turn_left:90;turn_left:90` |
| "back away slowly" | `backward:1.0` |

## Robot Command Reference

### Basic Movement
- `stop` - Stop all movement
- `forward` - Move forward indefinitely
- `forward:X` - Move forward X meters
- `backward:X` - Move backward X meters

### Turning
- `turn_left` - Turn left 90 degrees
- `turn_left:X` - Turn left X degrees
- `turn_right` - Turn right 90 degrees
- `turn_right:X` - Turn right X degrees

### Reactive Navigation
- `forward_until_close:DIRECTION:DISTANCE` - Move forward until obstacle detected
- `forward_until_close:DIRECTION:DISTANCE:ACTION` - Same, then execute action
- `backward_until_close:DIRECTION:DISTANCE` - Move backward until obstacle
- `turn_until_clear:DIRECTION:DISTANCE` - Turn until clear path found
- `follow_wall:SIDE:DISTANCE` - Follow wall at specified distance

### Directions
- `front`, `front_left`, `front_right`
- `left`, `right`
- `back`, `back_left`, `back_right`
- `any`

### Command Chaining
Multiple commands can be chained with semicolons:
```
forward:1.0;turn_left:90;forward:2.0
```

## Configuration

Edit `config.json` to customize the LLM settings:

```json
{
    "provider": "openai",
    "model": "gpt-4o-mini",
    "api_key_env_var": "OPENAI_API_KEY",
    "max_tokens": 150,
    "temperature": 0.1,
    "timeout_seconds": 30
}
```

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    NLP Interface Module                      │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─────────────┐    ┌──────────────────┐                   │
│  │   Natural   │───>│  NLPCommand      │                   │
│  │  Language   │    │  Translator      │                   │
│  │   Input     │    │                  │                   │
│  └─────────────┘    │  ┌────────────┐  │    ┌───────────┐  │
│                     │  │   Local    │──┼───>│  Robot    │  │
│                     │  │  Patterns  │  │    │  Command  │  │
│                     │  └────────────┘  │    └───────────┘  │
│                     │        │         │                   │
│                     │        v         │                   │
│                     │  ┌────────────┐  │                   │
│                     │  │    LLM     │  │                   │
│                     │  │   Client   │  │                   │
│                     │  └────────────┘  │                   │
│                     └──────────────────┘                   │
│                            │                               │
│                            v                               │
│                     ┌────────────┐                         │
│                     │  External  │                         │
│                     │  LLM API   │                         │
│                     └────────────┘                         │
└─────────────────────────────────────────────────────────────┘
```

## Files

- `__init__.py` - Module initialization and exports
- `config.json` - LLM configuration settings
- `llm_client.py` - LLM API client implementation
- `translator.py` - Main translation logic with local patterns
- `nlp_navigation_node.py` - ROS2 node for integration
- `README.md` - This documentation

## Extending

### Adding Local Patterns

Add new patterns to the `LOCAL_PATTERNS` list in `translator.py`:

```python
LOCAL_PATTERNS = [
    # ... existing patterns ...
    
    # New pattern: "spin around" -> turn_left:360
    (r"^spin\s+around$", "turn_left:360", None),
    
    # New pattern with extraction: "circle left for X meters"
    (
        r"^circle\s+left\s+for\s+(\d+(?:\.\d+)?)\s*(?:m(?:eters?)?)?$",
        "follow_wall:left:0.5:{0}",
        lambda m: (float(m.group(1)),),
    ),
]
```

### Supporting New LLM Providers

Add provider configuration to `config.json` and implement the translation method in `llm_client.py`.

## Troubleshooting

### API Key Not Set
```
ValueError: API key not set. Please set the appropriate environment variable.
```
**Solution**: Set the environment variable for your LLM provider.

### Network Error
```
RuntimeError: Network error: ...
```
**Solution**: Check your internet connection and API endpoint URL.

### Invalid Command Output
If the LLM returns an invalid command, the translator falls back to "stop" for safety.

## License

This module is part of the Flembot project and is released under the same license.
