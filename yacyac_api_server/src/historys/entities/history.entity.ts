import { User } from 'src/users/entities/user.entity';
import {
  Entity,
  PrimaryGeneratedColumn,
  Column,
  CreateDateColumn,
  ManyToOne,
  JoinColumn,
} from 'typeorm';

@Entity()
export class History {
  @PrimaryGeneratedColumn()
  id: number;

  @Column({ type: 'int', nullable: true })
  userId: number;

  @Column()
  accept: boolean;

  @ManyToOne(() => User)
  @JoinColumn({ name: 'userId' })
  user: User;

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

  @CreateDateColumn()
  createdDate: Date;
}
