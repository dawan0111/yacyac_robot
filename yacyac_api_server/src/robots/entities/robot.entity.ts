import { Entity, PrimaryGeneratedColumn, Column } from 'typeorm';

@Entity()
export class Robot {
  @PrimaryGeneratedColumn()
  id: number;

  @Column({ nullable: true, default: 0 })
  pill1: number;

  @Column({ nullable: true, default: 0 })
  pill2: number;

  @Column({ nullable: true, default: 0 })
  pill3: number;

  @Column({ nullable: true, default: 0 })
  pill4: number;

  @Column({ nullable: true, default: 0 })
  pill5: number;

  @Column({ nullable: true, default: 0 })
  pill6: number;

  @Column({ nullable: true, default: 0 })
  pill7: number;

  @Column({ nullable: true, default: 0 })
  pill8: number;
}
