import { Space, Table, Tag } from "antd";
import type { ColumnsType } from "antd/es/table";
import { useQuery } from "react-query";
import fetcher from "../apis/fetcher";
import dayjs from "dayjs";
import utc from "dayjs/plugin/utc";
import timezone from "dayjs/plugin/timezone";

dayjs.extend(utc);
dayjs.extend(timezone);

dayjs.tz.setDefault("Asia/Seoul");

const pillRender = (value: number) => <>{value} 개</>;

type PillData = {
  user: {
    name: string;
  };
  pill1: number;
  pill2: number;
  pill3: number;
  pill4: number;
  pill5: number;
  pill6: number;
  pill7: number;
  pill8: number;
  createdDate: string;
  address: number;
};

const columns: ColumnsType<PillData> = [
  {
    title: "이름",
    dataIndex: "user",
    key: "user",
    render: (value: any) => <>{value.name} 환자</>,
  },
  {
    title: "1번약",
    dataIndex: "pill1",
    key: "pill1",
    render: pillRender,
  },
  {
    title: "2번약",
    dataIndex: "pill2",
    key: "pill2",
    render: pillRender,
  },
  {
    title: "3번약",
    dataIndex: "pill3",
    key: "pill3",
    render: pillRender,
  },
  {
    title: "4번약",
    dataIndex: "pill4",
    key: "pill4",
    render: pillRender,
  },
  {
    title: "5번약",
    dataIndex: "pill5",
    key: "pill5",
    render: pillRender,
  },
  {
    title: "6번약",
    dataIndex: "pill6",
    key: "pill6",
    render: pillRender,
  },
  {
    title: "7번약",
    dataIndex: "pill7",
    key: "pill7",
    render: pillRender,
  },
  {
    title: "8번약",
    dataIndex: "pill8",
    key: "pill8",
    render: pillRender,
  },
  {
    title: "배급 시간",
    dataIndex: "createdDate",
    key: "createdDate",
    render: (value: string) => (
      <>
        {dayjs(value).add(9, "hour").format("YYYY년 MM월 DD일 HH시 mm분 ss초")}
      </>
    ),
  },
  {
    title: "배급 승인",
    dataIndex: "accept",
    key: "accept",
    render: (value: boolean) =>
      Boolean(value) ? (
        <Tag color="green">배급 완료</Tag>
      ) : (
        <Tag color="red">배급 거부</Tag>
      ),
  },
];

export default function Historys() {
  const { data, error, isLoading } = useQuery<PillData[]>("historys", () =>
    fetcher<PillData[]>("/historys")
  );

  console.log(data);

  return (
    <>
      <Table columns={columns} dataSource={data} />
    </>
  );
}
