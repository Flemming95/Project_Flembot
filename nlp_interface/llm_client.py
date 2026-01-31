"""
LLM Client for Natural Language Command Translation

This module provides a client interface for communicating with external LLM APIs
to translate natural language robot commands into the native command format.
"""

import json
import os
from typing import Optional
from pathlib import Path
import urllib.request
import urllib.error


class LLMClient:
    """
    Client for communicating with external LLM APIs.
    
    Supports multiple providers (OpenAI, Anthropic, Azure OpenAI) and handles
    API communication for translating natural language to robot commands.
    
    Attributes:
        provider: The LLM provider (e.g., 'openai', 'anthropic')
        model: The model name to use
        api_key: The API key for authentication
        max_tokens: Maximum tokens in the response
        temperature: Sampling temperature for the model
        timeout: Request timeout in seconds
    """
    
    # System prompt that defines the translation task
    SYSTEM_PROMPT = """You are a robot command translator. Your task is to convert natural language 
instructions into robot navigation commands.

Available commands:
- stop: Stop all movement
- forward: Move forward indefinitely  
- forward:X: Move forward for X meters (e.g., forward:2.0)
- backward:X: Move backward for X meters (e.g., backward:1.0)
- turn_left: Turn left 90 degrees
- turn_left:X: Turn left X degrees (e.g., turn_left:45)
- turn_right: Turn right 90 degrees  
- turn_right:X: Turn right X degrees (e.g., turn_right:90)
- forward_until_close:DIRECTION:DISTANCE: Move forward until obstacle at DIRECTION is within DISTANCE meters
- forward_until_close:DIRECTION:DISTANCE:ACTION: Same as above, then execute ACTION
- backward_until_close:DIRECTION:DISTANCE: Move backward until obstacle at DIRECTION is within DISTANCE meters
- turn_until_clear:DIRECTION:DISTANCE: Turn in DIRECTION until DISTANCE meters clear
- follow_wall:SIDE:DISTANCE: Follow wall on SIDE maintaining DISTANCE (e.g., follow_wall:left:0.5)
- follow_wall:SIDE:DISTANCE:LENGTH: Follow wall for LENGTH meters

Directions: front, front_left, front_right, left, right, back, back_left, back_right, any
Sides for wall following: left, right

Multiple commands can be chained with semicolons (;)
Example: forward:1.0;turn_left:90;forward:2.0

Rules:
1. Output ONLY the command string, no explanations
2. Use precise numeric values when specified
3. Chain commands with semicolons when multiple actions are needed
4. If the instruction is unclear, output the closest matching command
5. If no valid command matches, output: stop"""

    def __init__(
        self,
        provider: str = "huggingface",
        model: Optional[str] = None,
        api_key: Optional[str] = None,
        config_path: Optional[str] = None,
    ):
        """
        Initialize the LLM client.
        
        Args:
            provider: LLM provider name ('huggingface', 'openai', 'anthropic', 'azure_openai')
            model: Model name (optional, uses config default)
            api_key: API key (optional, reads from environment or config)
            config_path: Path to config file (optional)
        """
        # Load configuration
        self.config = self._load_config(config_path)
        
        # Set provider and model
        self.provider = provider or self.config.get("provider", "huggingface")
        
        if self.provider in self.config.get("alternative_providers", {}):
            provider_config = self.config["alternative_providers"][self.provider]
            default_model = provider_config.get("model")
            api_key_env = provider_config.get("api_key_env_var")
        else:
            default_model = self.config.get("model", "HuggingFaceTB/SmolLM3-3B")
            api_key_env = self.config.get("api_key_env_var", "HF_ACCESS_TOKEN")
        
        self.model = model or default_model
        
        # Try to get API key from: 1) parameter, 2) environment, 3) config file
        if api_key:
            self.api_key = api_key
        else:
            self.api_key = os.environ.get(api_key_env, "")
            if not self.api_key:
                # Fall back to api_key in config (for convenience)
                self.api_key = self.config.get("api_key", "")
        
        # Get other settings
        self.max_tokens = self.config.get("max_tokens", 150)
        self.temperature = self.config.get("temperature", 0.1)
        self.timeout = self.config.get("timeout_seconds", 30)
        
    def _load_config(self, config_path: Optional[str] = None) -> dict:
        """Load configuration from JSON file."""
        if config_path is None:
            config_path = Path(__file__).parent / "config.json"
        
        try:
            with open(config_path, "r") as f:
                return json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            return {}
    
    def translate(self, natural_language_input: str) -> str:
        """
        Translate natural language to robot command.
        
        Args:
            natural_language_input: The natural language instruction
            
        Returns:
            The translated robot command string
            
        Raises:
            ValueError: If the API key is not set
            RuntimeError: If the API request fails
        """
        if not self.api_key:
            raise ValueError(
                f"API key not set. Please set the appropriate environment variable "
                f"or pass the api_key parameter."
            )
        
        if self.provider == "huggingface":
            return self._translate_huggingface(natural_language_input)
        elif self.provider == "openai" or self.provider == "azure_openai":
            return self._translate_openai(natural_language_input)
        elif self.provider == "anthropic":
            return self._translate_anthropic(natural_language_input)
        else:
            raise ValueError(f"Unsupported provider: {self.provider}")
    
    def _translate_huggingface(self, user_input: str) -> str:
        """Translate using HuggingFace Inference API."""
        # Use the serverless inference API
        url = f"https://router.huggingface.co/hf-inference/models/{self.model}/v1/chat/completions"
        headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {self.api_key}",
        }
        
        payload = {
            "model": self.model,
            "messages": [
                {"role": "system", "content": self.SYSTEM_PROMPT},
                {"role": "user", "content": user_input},
            ],
            "max_tokens": self.max_tokens,
            "temperature": self.temperature,
        }
        
        return self._make_request(url, headers, payload, "openai")
    
    def _translate_openai(self, user_input: str) -> str:
        """Translate using OpenAI API."""
        if self.provider == "azure_openai":
            endpoint = os.environ.get("AZURE_OPENAI_ENDPOINT", "")
            url = f"{endpoint}/openai/deployments/{self.model}/chat/completions?api-version=2024-02-15-preview"
            headers = {
                "Content-Type": "application/json",
                "api-key": self.api_key,
            }
        else:
            url = "https://api.openai.com/v1/chat/completions"
            headers = {
                "Content-Type": "application/json",
                "Authorization": f"Bearer {self.api_key}",
            }
        
        payload = {
            "model": self.model,
            "messages": [
                {"role": "system", "content": self.SYSTEM_PROMPT},
                {"role": "user", "content": user_input},
            ],
            "max_tokens": self.max_tokens,
            "temperature": self.temperature,
        }
        
        return self._make_request(url, headers, payload, "openai")
    
    def _translate_anthropic(self, user_input: str) -> str:
        """Translate using Anthropic API."""
        url = "https://api.anthropic.com/v1/messages"
        headers = {
            "Content-Type": "application/json",
            "x-api-key": self.api_key,
            "anthropic-version": "2023-06-01",
        }
        
        payload = {
            "model": self.model,
            "max_tokens": self.max_tokens,
            "system": self.SYSTEM_PROMPT,
            "messages": [
                {"role": "user", "content": user_input},
            ],
        }
        
        return self._make_request(url, headers, payload, "anthropic")
    
    def _make_request(
        self, url: str, headers: dict, payload: dict, response_format: str
    ) -> str:
        """Make HTTP request to LLM API."""
        data = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(url, data=data, headers=headers, method="POST")
        
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as response:
                response_data = json.loads(response.read().decode("utf-8"))
                
                if response_format == "openai":
                    return response_data["choices"][0]["message"]["content"].strip()
                elif response_format == "anthropic":
                    return response_data["content"][0]["text"].strip()
                else:
                    raise ValueError(f"Unknown response format: {response_format}")
                    
        except urllib.error.HTTPError as e:
            error_body = e.read().decode("utf-8") if e.fp else ""
            raise RuntimeError(
                f"API request failed with status {e.code}: {error_body}"
            ) from e
        except urllib.error.URLError as e:
            raise RuntimeError(f"Network error: {e.reason}") from e
        except json.JSONDecodeError as e:
            raise RuntimeError(f"Failed to parse API response: {e}") from e


def main():
    """Demo function to test the LLM client."""
    import sys
    
    # Check if API key is available (HuggingFace is now the default)
    api_key = os.environ.get("HF_ACCESS_TOKEN", "")
    if not api_key:
        print("Warning: HF_ACCESS_TOKEN environment variable not set.")
        print("Please set it to use the LLM translation feature.")
        print("\nExample usage with API key:")
        print('  export HF_ACCESS_TOKEN="your-huggingface-token"')
        print("  python -m nlp_interface.llm_client")
        return
    
    client = LLMClient()
    print(f"Using provider: {client.provider}")
    print(f"Using model: {client.model}")
    
    test_inputs = [
        "move forward two meters",
        "turn left ninety degrees",
        "go backwards half a meter",
        "stop the robot",
        "move forward until you reach a wall then turn around",
    ]
    
    print("\nLLM Client Demo")
    print("=" * 50)
    
    for user_input in test_inputs:
        try:
            result = client.translate(user_input)
            print(f"\nInput:  '{user_input}'")
            print(f"Output: '{result}'")
        except Exception as e:
            print(f"\nInput:  '{user_input}'")
            print(f"Error:  {e}")


if __name__ == "__main__":
    main()
