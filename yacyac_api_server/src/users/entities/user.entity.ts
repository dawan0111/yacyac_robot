import { Entity, PrimaryGeneratedColumn, Column, BaseEntity } from 'typeorm';

@Entity()
export class User extends BaseEntity {
  @PrimaryGeneratedColumn()
  id: number;

  @Column()
  name: string;

  @Column('varchar', { length: 1000, nullable: true })
  photo: string;

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
