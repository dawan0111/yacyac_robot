import { Injectable } from '@nestjs/common';
import { InjectRepository } from '@nestjs/typeorm';
import { Repository } from 'typeorm';
import { History } from './entities/history.entity';
import { CreateHistoryDto } from './dto/create-history.dto';
import { UpdateHistoryDto } from './dto/update-history.dto';

@Injectable()
export class HistorysService {
  constructor(
    @InjectRepository(History)
    private readonly historyRepository: Repository<History>,
  ) {}

  async create(historyData: CreateHistoryDto): Promise<History> {
    const history = this.historyRepository.create(historyData);
    return await this.historyRepository.save(history);
  }

  async findAll(): Promise<History[]> {
    return await this.historyRepository.find({
      order: {
        createdDate: 'DESC',
      },
      relations: {
        user: true,
      },
    });
  }

  async findOne(id: number): Promise<History> {
    return await this.historyRepository.findOne({ where: { id } });
  }

  async getStat() {
    return this.historyRepository.count();
  }

  async update(id: number, historyData: UpdateHistoryDto): Promise<History> {
    await this.historyRepository.update(id, historyData);
    return await this.historyRepository.findOne({ where: { id } });
  }

  async remove(id: number): Promise<void> {
    await this.historyRepository.delete(id);
  }
}
