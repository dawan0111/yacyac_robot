import { Injectable } from '@nestjs/common';
import { InjectRepository } from '@nestjs/typeorm';
import { Repository } from 'typeorm';
import { User } from './entities/user.entity'; // User 엔티티 경로로 수정
import { CreateUserDto } from './dto/create-user.dto'; // CreateUserDto 경로로 수정
import { UpdateUserDto } from './dto/update-user.dto'; // UpdateUserDto 경로로 수정

@Injectable()
export class UsersService {
  constructor(
    @InjectRepository(User) // User 엔티티로 수정
    private readonly userRepository: Repository<User>, // userRepository로 수정
  ) {}

  async create(userData: CreateUserDto): Promise<User> {
    // 메서드 파라미터 이름 수정
    const user = this.userRepository.create(userData); // userRepository로 수정
    return await this.userRepository.save(user); // userRepository로 수정
  }

  async findAll(): Promise<User[]> {
    // 반환 타입 수정
    return await this.userRepository.find(); // userRepository로 수정
  }

  async findOne(id: number): Promise<User> {
    // 반환 타입 및 파라미터 이름 수정
    return await this.userRepository.findOne({ where: { id } }); // userRepository로 수정
  }

  async update(id: number, userData: UpdateUserDto): Promise<User> {
    // 반환 타입 및 파라미터 이름 수정
    await this.userRepository.update(id, userData); // userRepository로 수정
    return await this.userRepository.findOne({ where: { id } }); // userRepository로 수정
  }

  async remove(id: number): Promise<void> {
    // 파라미터 이름 수정
    await this.userRepository.delete(id); // userRepository로 수정
  }
}
