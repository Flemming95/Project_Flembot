"""
NLP Command Translator

This module provides the main interface for translating natural language
robot commands to the native command format. It combines local pattern matching
for common commands with LLM-based translation for complex inputs.
"""

import re
from typing import Optional, Tuple
from .llm_client import LLMClient


class NLPCommandTranslator:
    """
    Translator for converting natural language to robot navigation commands.
    
    Uses a hybrid approach:
    1. Local pattern matching for common/simple commands (fast, no API needed)
    2. LLM-based translation for complex commands (requires API key)
    
    Attributes:
        llm_client: The LLM client for complex translations
        use_llm: Whether to use LLM for complex commands
    """
    
    # Pattern matching rules for simple commands
    # Each tuple: (regex pattern, command template, extraction function)
    LOCAL_PATTERNS = [
        # Stop commands
        (r"^stop(?:\s+the\s+robot)?$", "stop", None),
        (r"^halt$", "stop", None),
        (r"^emergency\s+stop$", "stop", None),
        
        # Forward commands with distance
        (
            r"^(?:move\s+)?forward\s+(\d+(?:\.\d+)?)\s*(?:m(?:eters?)?)?$",
            "forward:{0}",
            lambda m: (float(m.group(1)),),
        ),
        (
            r"^(?:move\s+)?forward\s+(one|two|three|four|five|six|seven|eight|nine|ten|half)\s*(?:m(?:eters?)?)?$",
            "forward:{0}",
            lambda m: (_word_to_number(m.group(1)),),
        ),
        (
            r"^go\s+(?:straight\s+)?(?:forward\s+)?(\d+(?:\.\d+)?)\s*(?:m(?:eters?)?)?$",
            "forward:{0}",
            lambda m: (float(m.group(1)),),
        ),
        
        # Forward indefinitely
        (r"^(?:move\s+)?forward$", "forward", None),
        (r"^go\s+(?:straight|forward)$", "forward", None),
        
        # Backward commands with distance
        (
            r"^(?:move\s+)?back(?:wards?)?\s+(\d+(?:\.\d+)?)\s*(?:m(?:eters?)?)?$",
            "backward:{0}",
            lambda m: (float(m.group(1)),),
        ),
        (
            r"^(?:move\s+)?back(?:wards?)?\s+(one|two|three|four|five|six|seven|eight|nine|ten|half)\s*(?:m(?:eters?)?)?$",
            "backward:{0}",
            lambda m: (_word_to_number(m.group(1)),),
        ),
        (r"^reverse\s+(\d+(?:\.\d+)?)\s*(?:m(?:eters?)?)?$", "backward:{0}", lambda m: (float(m.group(1)),)),
        
        # Turn left commands with angle
        (
            r"^turn\s+left\s+(\d+)\s*(?:deg(?:rees?)?)?$",
            "turn_left:{0}",
            lambda m: (int(m.group(1)),),
        ),
        (
            r"^turn\s+left\s+(ninety|forty[\s-]?five|one\s*eighty|thirty|sixty|one\s*twenty)(?:\s*deg(?:rees?)?)?$",
            "turn_left:{0}",
            lambda m: (_angle_word_to_number(m.group(1)),),
        ),
        (r"^turn\s+left$", "turn_left", None),
        (r"^rotate\s+left$", "turn_left", None),
        (r"^left\s+turn$", "turn_left", None),
        
        # Turn right commands with angle
        (
            r"^turn\s+right\s+(\d+)\s*(?:deg(?:rees?)?)?$",
            "turn_right:{0}",
            lambda m: (int(m.group(1)),),
        ),
        (
            r"^turn\s+right\s+(ninety|forty[\s-]?five|one\s*eighty|thirty|sixty|one\s*twenty)(?:\s*deg(?:rees?)?)?$",
            "turn_right:{0}",
            lambda m: (_angle_word_to_number(m.group(1)),),
        ),
        (r"^turn\s+right$", "turn_right", None),
        (r"^rotate\s+right$", "turn_right", None),
        (r"^right\s+turn$", "turn_right", None),
        
        # Turn around (180 degrees)
        (r"^turn\s+around$", "turn_left:180", None),
        (r"^u[\s-]?turn$", "turn_left:180", None),
        
        # Wall following
        (
            r"^follow\s+(?:the\s+)?(?:left|right)\s+wall(?:\s+at\s+(\d+(?:\.\d+)?)\s*(?:m(?:eters?)?)?)?$",
            "follow_wall:{side}:{distance}",
            lambda m: {"side": "left" if "left" in m.group(0) else "right", "distance": m.group(1) or "0.5"},
        ),
    ]
    
    def __init__(
        self,
        use_llm: bool = True,
        llm_provider: str = "openai",
        llm_model: Optional[str] = None,
        api_key: Optional[str] = None,
    ):
        """
        Initialize the NLP command translator.
        
        Args:
            use_llm: Whether to use LLM for complex translations
            llm_provider: The LLM provider to use
            llm_model: The model name (optional)
            api_key: API key for the LLM provider (optional)
        """
        self.use_llm = use_llm
        self.llm_client: Optional[LLMClient] = None
        
        if use_llm:
            try:
                self.llm_client = LLMClient(
                    provider=llm_provider,
                    model=llm_model,
                    api_key=api_key,
                )
            except Exception:
                # LLM client initialization failed, will use local only
                self.llm_client = None
    
    def translate(self, natural_language_input: str) -> Tuple[str, str]:
        """
        Translate natural language to robot command.
        
        Args:
            natural_language_input: The natural language instruction
            
        Returns:
            Tuple of (command, method) where method is 'local' or 'llm'
        """
        # Normalize input
        normalized = natural_language_input.lower().strip()
        
        # Try local pattern matching first
        local_result = self._try_local_patterns(normalized)
        if local_result:
            return local_result, "local"
        
        # Fall back to LLM translation
        if self.use_llm and self.llm_client:
            try:
                llm_result = self.llm_client.translate(natural_language_input)
                # Validate the LLM result
                if self._validate_command(llm_result):
                    return llm_result, "llm"
            except Exception:
                pass
        
        # If all else fails, return stop as safe fallback
        return "stop", "fallback"
    
    def _try_local_patterns(self, text: str) -> Optional[str]:
        """Try to match input against local patterns."""
        for pattern, template, extractor in self.LOCAL_PATTERNS:
            match = re.match(pattern, text, re.IGNORECASE)
            if match:
                if extractor is None:
                    return template
                
                extracted = extractor(match)
                if isinstance(extracted, dict):
                    return template.format(**extracted)
                else:
                    return template.format(*extracted)
        
        return None
    
    def _validate_command(self, command: str) -> bool:
        """Validate that a command string is well-formed."""
        if not command:
            return False
        
        # Split by semicolons for chained commands
        parts = command.split(";")
        
        valid_actions = {
            "stop", "forward", "backward", "turn_left", "turn_right",
            "forward_until_close", "backward_until_close",
            "turn_until_clear", "follow_wall",
        }
        
        for part in parts:
            part = part.strip()
            if not part:
                continue
            
            # Extract action from command
            action = part.split(":")[0]
            if action not in valid_actions:
                return False
        
        return True
    
    def get_available_commands(self) -> str:
        """Return a string describing available commands."""
        return """
Available Robot Navigation Commands:
=====================================

Basic Movement:
  - stop                      Stop all movement
  - forward                   Move forward indefinitely
  - forward:X                 Move forward X meters (e.g., forward:2.0)
  - backward:X                Move backward X meters

Turning:
  - turn_left                 Turn left 90 degrees
  - turn_left:X               Turn left X degrees (e.g., turn_left:45)
  - turn_right                Turn right 90 degrees
  - turn_right:X              Turn right X degrees

Reactive Navigation:
  - forward_until_close:DIRECTION:DISTANCE
      Move forward until obstacle at DIRECTION is within DISTANCE meters
      Directions: front, front_left, front_right, left, right, back, back_left, back_right, any
      
  - forward_until_close:DIRECTION:DISTANCE:ACTION
      Same as above, then execute ACTION

  - turn_until_clear:DIRECTION:DISTANCE
      Turn in DIRECTION until DISTANCE meters clear

  - follow_wall:SIDE:DISTANCE
      Follow wall on SIDE maintaining DISTANCE (e.g., follow_wall:left:0.5)

Command Chaining:
  Multiple commands can be chained with semicolons
  Example: forward:1.0;turn_left:90;forward:2.0
"""


def _word_to_number(word: str) -> float:
    """Convert word number to float."""
    word_map = {
        "one": 1.0,
        "two": 2.0,
        "three": 3.0,
        "four": 4.0,
        "five": 5.0,
        "six": 6.0,
        "seven": 7.0,
        "eight": 8.0,
        "nine": 9.0,
        "ten": 10.0,
        "half": 0.5,
    }
    return word_map.get(word.lower(), 1.0)


def _angle_word_to_number(word: str) -> int:
    """Convert angle word to integer degrees."""
    word = word.lower().replace(" ", "").replace("-", "")
    angle_map = {
        "thirty": 30,
        "fortyfive": 45,
        "sixty": 60,
        "ninety": 90,
        "onetwenty": 120,
        "oneeighty": 180,
    }
    return angle_map.get(word, 90)


def main():
    """Demo function to test the translator."""
    translator = NLPCommandTranslator(use_llm=False)  # Local only for demo
    
    test_inputs = [
        "stop",
        "stop the robot",
        "move forward 2 meters",
        "forward two meters",
        "go straight 1.5 meters",
        "turn left",
        "turn left 45 degrees",
        "turn right ninety degrees",
        "turn around",
        "move backwards 1 meter",
        "follow the left wall",
        "follow right wall at 0.3 meters",
    ]
    
    print("NLP Command Translator Demo (Local Patterns Only)")
    print("=" * 55)
    
    for user_input in test_inputs:
        command, method = translator.translate(user_input)
        print(f"\nInput:   '{user_input}'")
        print(f"Command: '{command}' (via {method})")
    
    print("\n" + "=" * 55)
    print(translator.get_available_commands())


if __name__ == "__main__":
    main()
