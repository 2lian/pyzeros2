import logging
from dataclasses import dataclass, field
from typing import Any, Dict, Final, Generic, NamedTuple, Optional, Tuple, TypeVar

import ros_z_py
from asyncio_for_robotics.core.sub import BaseSub, _MsgType

from .session import ZNode, auto_session
from .utils import QOS_DEFAULT, TopicInfo

logger = logging.getLogger(__name__)


class Sub(BaseSub[_MsgType]):
    def __init__(
        self,
        msg_type: type[_MsgType],
        topic: str,
        qos_profile: ros_z_py.QosProfile = QOS_DEFAULT,
        session: Optional[ZNode] = None,
    ) -> None:
        self.session = self._resolve_session(session)
        self.topic_info = TopicInfo(topic=topic, msg_type=msg_type, qos=qos_profile)
        self.sub = self._resolve_sub(self.topic_info)
        super().__init__()

    @property
    def name(self) -> str:
        return f"Sub rzp-{self.topic_info.topic}"

    def _resolve_session(self, session: Optional[ZNode]) -> ZNode:
        if session is None:
            return auto_session()
        else:
            return session

    def _resolve_sub(self, topic_info: TopicInfo):
        return self.session.create_subscriber(
            topic_info.topic,
            topic_info.msg_type,
            topic_info.qos,
            self.callback_for_sub,
        )

    def callback_for_sub(self, sample: _MsgType):
        if self._closed.is_set():
            return
        try:
            healty = self.input_data(sample)
            if not healty:
                self.close()
        except Exception as e:
            logger.error(e)

    def close(self):
        raise NotImplementedError
        if not self._closed.is_set():
            if not self.session.is_closed():
                self.sub.undeclare()
            else:
                logger.debug("Zenoh session already closed for %s", self.name)
        super().close()
