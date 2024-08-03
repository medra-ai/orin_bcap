from __future__ import annotations
import numpy
import typing
__all__ = ['BCAP_HRESULT', 'BCAP_VARIANT', 'Deg2Rad', 'DensoController', 'Rad2Deg', 'RobotTrajectory', 'VRad2Deg', 'bCapException']
class BCAP_HRESULT:
    """
    Members:
    
      BCAP_S_OK
    
      BCAP_E_NOTIMPL
    
      BCAP_E_ABORT
    
      BCAP_E_FAIL
    
      BCAP_E_UNEXPECTED
    
      BCAP_E_INVALIDRCVPACKET
    
      BCAP_E_INVALIDSNDPACKET
    
      BCAP_E_INVALIDARGTYPE
    
      BCAP_E_ROBOTISBUSY
    
      BCAP_E_INVALIDCOMMAND
    
      BCAP_E_PACKETSIZEOVER
    
      BCAP_E_ARGSIZEOVER
    
      BCAP_E_ACCESSDENIED
    
      BCAP_E_HANDLE
    
      BCAP_E_OUTOFMEMORY
    
      BCAP_E_INVALIDARG
    """
    BCAP_E_ABORT: typing.ClassVar[BCAP_HRESULT]  # value = <BCAP_HRESULT.BCAP_E_ABORT: 2147500036>
    BCAP_E_ACCESSDENIED: typing.ClassVar[BCAP_HRESULT]  # value = <BCAP_HRESULT.BCAP_E_ACCESSDENIED: 2147942405>
    BCAP_E_ARGSIZEOVER: typing.ClassVar[BCAP_HRESULT]  # value = <BCAP_HRESULT.BCAP_E_ARGSIZEOVER: 2147549202>
    BCAP_E_FAIL: typing.ClassVar[BCAP_HRESULT]  # value = <BCAP_HRESULT.BCAP_E_FAIL: 2147500037>
    BCAP_E_HANDLE: typing.ClassVar[BCAP_HRESULT]  # value = <BCAP_HRESULT.BCAP_E_HANDLE: 2147942406>
    BCAP_E_INVALIDARG: typing.ClassVar[BCAP_HRESULT]  # value = <BCAP_HRESULT.BCAP_E_INVALIDARG: 2147942487>
    BCAP_E_INVALIDARGTYPE: typing.ClassVar[BCAP_HRESULT]  # value = <BCAP_HRESULT.BCAP_E_INVALIDARGTYPE: 2147549187>
    BCAP_E_INVALIDCOMMAND: typing.ClassVar[BCAP_HRESULT]  # value = <BCAP_HRESULT.BCAP_E_INVALIDCOMMAND: 2147549189>
    BCAP_E_INVALIDRCVPACKET: typing.ClassVar[BCAP_HRESULT]  # value = <BCAP_HRESULT.BCAP_E_INVALIDRCVPACKET: 2147549185>
    BCAP_E_INVALIDSNDPACKET: typing.ClassVar[BCAP_HRESULT]  # value = <BCAP_HRESULT.BCAP_E_INVALIDSNDPACKET: 2147549186>
    BCAP_E_NOTIMPL: typing.ClassVar[BCAP_HRESULT]  # value = <BCAP_HRESULT.BCAP_E_NOTIMPL: 2147500033>
    BCAP_E_OUTOFMEMORY: typing.ClassVar[BCAP_HRESULT]  # value = <BCAP_HRESULT.BCAP_E_OUTOFMEMORY: 2147942414>
    BCAP_E_PACKETSIZEOVER: typing.ClassVar[BCAP_HRESULT]  # value = <BCAP_HRESULT.BCAP_E_PACKETSIZEOVER: 2147549201>
    BCAP_E_ROBOTISBUSY: typing.ClassVar[BCAP_HRESULT]  # value = <BCAP_HRESULT.BCAP_E_ROBOTISBUSY: 2147549188>
    BCAP_E_UNEXPECTED: typing.ClassVar[BCAP_HRESULT]  # value = <BCAP_HRESULT.BCAP_E_UNEXPECTED: 2147549183>
    BCAP_S_OK: typing.ClassVar[BCAP_HRESULT]  # value = <BCAP_HRESULT.BCAP_S_OK: 0>
    __members__: typing.ClassVar[dict[str, BCAP_HRESULT]]  # value = {'BCAP_S_OK': <BCAP_HRESULT.BCAP_S_OK: 0>, 'BCAP_E_NOTIMPL': <BCAP_HRESULT.BCAP_E_NOTIMPL: 2147500033>, 'BCAP_E_ABORT': <BCAP_HRESULT.BCAP_E_ABORT: 2147500036>, 'BCAP_E_FAIL': <BCAP_HRESULT.BCAP_E_FAIL: 2147500037>, 'BCAP_E_UNEXPECTED': <BCAP_HRESULT.BCAP_E_UNEXPECTED: 2147549183>, 'BCAP_E_INVALIDRCVPACKET': <BCAP_HRESULT.BCAP_E_INVALIDRCVPACKET: 2147549185>, 'BCAP_E_INVALIDSNDPACKET': <BCAP_HRESULT.BCAP_E_INVALIDSNDPACKET: 2147549186>, 'BCAP_E_INVALIDARGTYPE': <BCAP_HRESULT.BCAP_E_INVALIDARGTYPE: 2147549187>, 'BCAP_E_ROBOTISBUSY': <BCAP_HRESULT.BCAP_E_ROBOTISBUSY: 2147549188>, 'BCAP_E_INVALIDCOMMAND': <BCAP_HRESULT.BCAP_E_INVALIDCOMMAND: 2147549189>, 'BCAP_E_PACKETSIZEOVER': <BCAP_HRESULT.BCAP_E_PACKETSIZEOVER: 2147549201>, 'BCAP_E_ARGSIZEOVER': <BCAP_HRESULT.BCAP_E_ARGSIZEOVER: 2147549202>, 'BCAP_E_ACCESSDENIED': <BCAP_HRESULT.BCAP_E_ACCESSDENIED: 2147942405>, 'BCAP_E_HANDLE': <BCAP_HRESULT.BCAP_E_HANDLE: 2147942406>, 'BCAP_E_OUTOFMEMORY': <BCAP_HRESULT.BCAP_E_OUTOFMEMORY: 2147942414>, 'BCAP_E_INVALIDARG': <BCAP_HRESULT.BCAP_E_INVALIDARG: 2147942487>}
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class BCAP_VARIANT:
    Arrays: int
    CharValue: int
    Data: int
    DoubleArray: numpy.ndarray[numpy.float64]
    DoubleValue: float
    FloatArray: numpy.ndarray[numpy.float32]
    FloatValue: float
    LongValue: int
    ShortValue: int
    String: str
    Type: int
    def __init__(self) -> None:
        ...
class DensoController:
    def ChangeTool(self, arg0: str) -> BCAP_HRESULT:
        ...
    def CommandFromVector(self, arg0: list[float]) -> str:
        ...
    def GetCurJnt(self) -> list[float]:
        ...
    def GetMountingCalib(self, arg0: str) -> list[float]:
        ...
    def ManualReset(self) -> BCAP_HRESULT:
        ...
    def RadVectorFromVNT(self, arg0: BCAP_VARIANT) -> list[float]:
        ...
    def SetExtSpeed(self, arg0: str) -> BCAP_HRESULT:
        ...
    def SetTcpLoad(self, arg0: int) -> BCAP_HRESULT:
        ...
    def VNTFromRadVector(self, arg0: list[float]) -> BCAP_VARIANT:
        ...
    def VNTFromVector(self, arg0: list[float]) -> BCAP_VARIANT:
        ...
    def VectorFromVNT(self, arg0: BCAP_VARIANT) -> list[float]:
        ...
    def __init__(self) -> None:
        ...
    def bCapClose(self) -> None:
        ...
    def bCapControllerConnect(self) -> None:
        ...
    def bCapControllerDisconnect(self) -> None:
        ...
    def bCapEnterProcess(self) -> None:
        ...
    def bCapExitProcess(self) -> None:
        ...
    def bCapGetRobot(self) -> None:
        ...
    def bCapMotor(self, arg0: bool) -> BCAP_HRESULT:
        ...
    def bCapOpen(self) -> None:
        ...
    def bCapReleaseRobot(self) -> None:
        ...
    def bCapRobotExecute(self, arg0: str, arg1: str) -> BCAP_HRESULT:
        ...
    def bCapRobotMove(self, arg0: str, arg1: str) -> BCAP_HRESULT:
        ...
    def bCapServiceStart(self) -> None:
        ...
    def bCapServiceStop(self) -> None:
        ...
    def bCapSlvChangeMode(self, arg0: str) -> BCAP_HRESULT:
        ...
    def bCapSlvMove(self, arg0: BCAP_VARIANT, arg1: BCAP_VARIANT) -> BCAP_HRESULT:
        ...
    def executeServoTrajectory(self, arg0: RobotTrajectory) -> int:
        ...
    @property
    def iSockFD(self) -> int:
        ...
    @property
    def lhController(self) -> int:
        ...
    @property
    def lhRobot(self) -> int:
        ...
    @property
    def server_ip_address(self) -> str:
        ...
    @property
    def server_port_num(self) -> int:
        ...
class RobotTrajectory:
    trajectory: list[list[float]]
    def __init__(self) -> None:
        ...
    def size(self) -> int:
        ...
    @property
    def dimension(self) -> int:
        ...
class bCapException:
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, arg0: str, arg1: int) -> None:
        ...
def Deg2Rad(arg0: float) -> float:
    ...
def Rad2Deg(arg0: float) -> float:
    ...
def VRad2Deg(arg0: list[float]) -> list[float]:
    ...
