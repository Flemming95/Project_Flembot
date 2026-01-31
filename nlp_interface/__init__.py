"""
NLP Interface for Natural Language Robot Navigation

This module provides a natural language interface for controlling the robot
using LLM-based command translation. Natural language inputs are translated
to the robot's native navigation command format.

Example:
    >>> from nlp_interface import NLPCommandTranslator
    >>> translator = NLPCommandTranslator()
    >>> command = translator.translate("move forward two meters")
    >>> print(command)  # "forward:2.0"
"""

from .translator import NLPCommandTranslator
from .llm_client import LLMClient

__all__ = ["NLPCommandTranslator", "LLMClient"]
__version__ = "0.1.0"
