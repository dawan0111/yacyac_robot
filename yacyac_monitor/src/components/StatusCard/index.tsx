import StatusCardStyled from "./styled";

export interface StatusCardProps {
  title: React.ReactNode;
  content: React.ReactNode;
}

export default function StatusCard({ title, content }: StatusCardProps) {
  return (
    <StatusCardStyled>
      <div className="title">{title}</div>
      <div className="content">{content}</div>
    </StatusCardStyled>
  );
}
