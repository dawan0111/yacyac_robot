import React, { useState, useEffect } from "react";
import { ClockCircleOutlined } from "@ant-design/icons";
import dayjs from "dayjs";

function Clock() {
  const [currentTime, setCurrentTime] = useState(dayjs());

  useEffect(() => {
    const intervalId = setInterval(() => {
      setCurrentTime(dayjs());
    }, 1000); // 1초마다 업데이트

    return () => {
      clearInterval(intervalId);
    };
  }, []);

  const formattedTime = currentTime.format("YYYY년 MM월 DD일 HH시 mm분 ss초");

  return (
    <div>
      <p>
        <ClockCircleOutlined /> {formattedTime}
      </p>
    </div>
  );
}

export default Clock;
