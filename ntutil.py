from typing import Any, Dict, Generic, List, Type, TypeVar

import ntcore
import wpilib
import wpiutil.log


nt = ntcore.NetworkTableInstance.getDefault()

T = TypeVar("T")

class Topic(Generic[T]):
    """
    A single entry in NetworkTables that can be read from or written to.
    """

    def __init__(self, topic, default: T):
        """
        Do not create instances of Topic() directly. Instead, call the helper
        functions like `getIntegerTopic` and `getStructTopic`.
        """
        self.publisher = topic.publish()
        self.subscriber = topic.subscribe(default)

    def get(self, defaultValue: T | None = None) -> T:
        """
        Gets the most recent value published to the topic, or `defaultValue` if
        no value has been published.
        """
        if defaultValue is None:
            return self.subscriber.get()
        else:
            return self.subscriber.get(defaultValue)

    def set(self, value: T) -> None:
        """
        Sets the topic's value, publishing it to NetworkTables.
        """
        self.publisher.set(value)

    def setDefault(self, value: T) -> None:
        """
        Set's the default value of the topic, to be returned if no value has
        been published.
        """
        self.publisher.setDefault(value)


class DummyTopic(Topic[T], Generic[T]):
    """
    A phony topic that publishes and receives nothing. Useful as a default if
    you want NetworkTables logging to be optional.
    """

    def __init__(self, default: T | None = None):
        pass

    def get(self, defaultValue: T | None = None) -> T:
        # Returns None if no defaultValue is provided, but you know, who cares.
        # This is under-specified in the underlying library too.
        return defaultValue  # type: ignore

    def set(self, value: T) -> None:
        pass

    def setDefault(self, value: T) -> None:
        pass


class Folder:
    """
    An object that can be used to create Topics with a specific path. Useful
    when you want a group of topics for a specific object.

    ```
    nt = ntutil.Folder("MySubsystem")
    armAngleTopic = nt.getFloatTopic("ArmAngle") # Creates MySubsystem/ArmAngle
    clawOpenTopic = nt.getBooleanTopic("ClawOpen") # Creates MySubsystem/ClawOpen
    ```
    
    You can also create subfolders by calling `getFolder` again.
    """
    def __init__(self, prefix: str):
        """
        Creates a folder with a given path prefix. Do not call this directly;
        instead, call `getFolder`.
        """
        self.prefix = prefix

    def getFolder(self, name: str):
        """
        Creates a subfolder of the current folder.
        """
        return Folder(self.prefix + "/" + name)

    def getBooleanArrayTopic(self, name: str, defaultValue: List[bool] = []) -> Topic[List[bool]]:
        return Topic(nt.getBooleanArrayTopic(self.prefix + "/" + name), defaultValue)

    def getBooleanTopic(self, name: str, defaultValue: bool = False) -> Topic[bool]:
        return Topic(nt.getBooleanTopic(self.prefix + "/" + name), defaultValue)

    def getFloatArrayTopic(self, name: str, defaultValue: List[float] = []) -> Topic[List[float]]:
        return Topic(nt.getFloatArrayTopic(self.prefix + "/" + name), defaultValue)

    def getFloatTopic(self, name: str, defaultValue: float = 0) -> Topic[float]:
        return Topic(nt.getFloatTopic(self.prefix + "/" + name), defaultValue)

    def getIntegerArrayTopic(self, name: str, defaultValue: List[int] = []) -> Topic[List[int]]:
        return Topic(nt.getIntegerArrayTopic(self.prefix + "/" + name), defaultValue)

    def getIntegerTopic(self, name: str, defaultValue: int = 0) -> Topic[int]:
        return Topic(nt.getIntegerTopic(self.prefix + "/" + name), defaultValue)

    def getStringArrayTopic(self, name: str, defaultValue: List[str] = []) -> Topic[List[str]]:
        return Topic(nt.getStringArrayTopic(self.prefix + "/" + name), defaultValue)

    def getStringTopic(self, name: str, defaultValue: str = "") -> Topic[str]:
        return Topic(nt.getStringTopic(self.prefix + "/" + name), defaultValue)

    def getStructArrayTopic(self, name: str, type: Type[T], defaultValue: List[T] = []) -> Topic[List[T]]:
        return Topic(nt.getStructArrayTopic(self.prefix + "/" + name, type), defaultValue)

    def getStructTopic(self, name: str, type: Type[T], defaultValue: T | None = None) -> Topic[T]:
        if defaultValue is None:
            defaultValue = type()
        return Topic(nt.getStructTopic(self.prefix + "/" + name, type), defaultValue)


class DummyFolder(Folder):
    """
    A phony folder that creates phony topics. Useful as a default if you want
    NetworkTables logging to be optional.
    """

    def __init__(self, prefix: str = "DUMMY"):
        self.prefix = prefix

    def getFolder(self, name: str):
        return DummyFolder(self.prefix + "/" + name)

    def getBooleanArrayTopic(self, name: str, defaultValue: List[bool] = []) -> Topic[List[bool]]:
        return DummyTopic(defaultValue)

    def getBooleanTopic(self, name: str, defaultValue: bool = False) -> Topic[bool]:
        return DummyTopic(defaultValue)

    def getFloatArrayTopic(self, name: str, defaultValue: List[float] = []) -> Topic[List[float]]:
        return DummyTopic(defaultValue)
    
    def getFloatTopic(self, name: str, defaultValue: float = 0) -> Topic[float]:
        return DummyTopic(defaultValue)
    
    def getIntegerArrayTopic(self, name: str, defaultValue: List[int] = []) -> Topic[List[int]]:
        return DummyTopic(defaultValue)

    def getIntegerTopic(self, name: str, defaultValue: int = 0) -> Topic[int]:
        return DummyTopic(defaultValue)

    def getStringArrayTopic(self, name: str, defaultValue: List[str] = []) -> Topic[List[str]]:
        return DummyTopic(defaultValue)

    def getStringTopic(self, name: str, defaultValue: str = "") -> Topic[str]:
        return DummyTopic(defaultValue)

    def getStructArrayTopic(self, name: str, type: Type[T], defaultValue: List[T] = []) -> Topic[List[T]]:
        return DummyTopic(defaultValue)

    def getStructTopic(self, name: str, type: Type[T], defaultValue: T | None = None) -> Topic[T]:
        if defaultValue is None:
            defaultValue = type()
        return DummyTopic(defaultValue)


_globalFolder = Folder("")

def getFolder(name: str) -> Folder:
    return Folder("/" + name)

def getBooleanArrayTopic(name: str, defaultValue: List[bool] = []):
    return _globalFolder.getBooleanArrayTopic(name, defaultValue)

def getBooleanTopic(name: str, defaultValue: bool = False) -> Topic[bool]:
    return _globalFolder.getBooleanTopic(name, defaultValue)

def getFloatArrayTopic(name: str, defaultValue: List[float] = []) -> Topic[List[float]]:
    return _globalFolder.getFloatArrayTopic(name, defaultValue)

def getFloatTopic(name: str, defaultValue: float = 0) -> Topic[float]:
    return _globalFolder.getFloatTopic(name, defaultValue)

def getIntegerArrayTopic(name: str, defaultValue: List[int] = []) -> Topic[List[int]]:
    return _globalFolder.getIntegerArrayTopic(name, defaultValue)

def getIntegerTopic(name: str, defaultValue: int = 0) -> Topic[int]:
    return _globalFolder.getIntegerTopic(name, defaultValue)

def getStringArrayTopic(name: str, defaultValue: List[str] = []) -> Topic[List[str]]:
    return _globalFolder.getStringArrayTopic(name, defaultValue)

def getStringTopic(name: str, defaultValue: str = "") -> Topic[str]:
    return _globalFolder.getStringTopic(name, defaultValue)

def getStructArrayTopic(name: str, type: Type[T], defaultValue: List[T] = []) -> Topic[List[T]]:
    return _globalFolder.getStructArrayTopic(name, type, defaultValue)

def getStructTopic(name: str, type: Type[T], defaultValue: T | None = None) -> Topic[T]:
    return _globalFolder.getStructTopic(name, type, defaultValue)


# WPILib logging utilities

def log(msg: Any):
    """
    Logs a message to both stdout and the /messages entry of NetworkTables
    (visible when viewing log files).
    """
    wpilib.DataLogManager.log(f"{msg}")

_originalAlertText: Dict[wpilib.Alert, str] = {}

def logAlert(alert: wpilib.Alert, msg: Any):
    """
    Enables an Alert and also logs an additional message. Useful for indicating
    a specific value that caused an error, or to provide more context. Consider
    using this whenever you would instead use a simple `ntutil.log`.

    To avoid spam, this method will only log when the message changes.
    """
    global _originalAlertText
    
    if alert not in _originalAlertText:
        _originalAlertText[alert] = alert.getText()
    
    fullMsg = f"{_originalAlertText[alert]}: {msg}"
    if alert.getText() != fullMsg:
        log(fullMsg)
        alert.setText(fullMsg)
    alert.set(True)

def getBooleanArrayLog(name: str):
    return wpiutil.log.BooleanArrayLogEntry(wpilib.DataLogManager.getLog(), name)

def getBooleanLog(name: str):
    return wpiutil.log.BooleanLogEntry(wpilib.DataLogManager.getLog(), name)

def getDoubleArrayLog(name: str):
    return wpiutil.log.DoubleArrayLogEntry(wpilib.DataLogManager.getLog(), name)

def getDoubleLog(name: str):
    return wpiutil.log.DoubleLogEntry(wpilib.DataLogManager.getLog(), name)

def getFloatArrayLog(name: str):
    return wpiutil.log.FloatArrayLogEntry(wpilib.DataLogManager.getLog(), name)

def getFloatLog(name: str):
    return wpiutil.log.FloatLogEntry(wpilib.DataLogManager.getLog(), name)

def getIntegerArrayLog(name: str):
    return wpiutil.log.IntegerArrayLogEntry(wpilib.DataLogManager.getLog(), name)

def getIntegerLog(name: str):
    return wpiutil.log.IntegerLogEntry(wpilib.DataLogManager.getLog(), name)

def getStringArrayLog(name: str):
    return wpiutil.log.StringArrayLogEntry(wpilib.DataLogManager.getLog(), name)

def getStringLog(name: str):
    return wpiutil.log.StringLogEntry(wpilib.DataLogManager.getLog(), name)

def getStructArrayLog(name: str, type: type):
    return wpiutil.log.StructArrayLogEntry(wpilib.DataLogManager.getLog(), name, type)

def getStructLog(name: str, type: type):
    return wpiutil.log.StructLogEntry(wpilib.DataLogManager.getLog(), name, type)
