import { randomUUID } from "node:crypto";

import { config } from "../config.js";
import { PostgresUploadStore } from "./postgres-store.js";

export type UploadStatus = "pending" | "completed";

export type UploadFileRecord = {
  id: string;
  uploadId: string;
  originalName: string;
  objectName: string;
  contentType: string;
  sizeBytes: number;
  createdAt: Date;
};

export type UploadRecord = {
  id: string;
  userId: string;
  prompt: string;
  status: UploadStatus;
  uploadPrefix: string;
  resultObjectName?: string;
  resultFileName?: string;
  resultContentType?: string;
  filesUploadedAt?: Date;
  completedAt?: Date;
  createdAt: Date;
  updatedAt: Date;
  files: UploadFileRecord[];
};

export type CreateUploadFileInput = {
  id: string;
  originalName: string;
  objectName: string;
  contentType: string;
  sizeBytes: number;
};

export type CreateUploadInput = {
  id: string;
  userId: string;
  prompt: string;
  uploadPrefix: string;
  files: CreateUploadFileInput[];
};

export interface UploadStore {
  createUpload(input: CreateUploadInput): Promise<UploadRecord>;
  listUserUploads(userId: string): Promise<UploadRecord[]>;
  getUserUpload(userId: string, uploadId: string): Promise<UploadRecord | null>;
  markFilesUploaded(userId: string, uploadId: string): Promise<void>;
}

const now = () => new Date();

export class MemoryUploadStore implements UploadStore {
  private readonly uploads = new Map<string, UploadRecord>();

  async createUpload(input: CreateUploadInput): Promise<UploadRecord> {
    const createdAt = now();
    const upload: UploadRecord = {
      id: input.id,
      userId: input.userId,
      prompt: input.prompt,
      status: "pending",
      uploadPrefix: input.uploadPrefix,
      createdAt,
      updatedAt: createdAt,
      files: input.files.map((file) => ({
        id: file.id,
        uploadId: input.id,
        originalName: file.originalName,
        objectName: file.objectName,
        contentType: file.contentType,
        sizeBytes: file.sizeBytes,
        createdAt
      }))
    };

    this.uploads.set(upload.id, upload);
    return upload;
  }

  async listUserUploads(userId: string): Promise<UploadRecord[]> {
    return [...this.uploads.values()]
      .filter((upload) => upload.userId === userId)
      .sort((a, b) => b.createdAt.getTime() - a.createdAt.getTime());
  }

  async getUserUpload(userId: string, uploadId: string): Promise<UploadRecord | null> {
    const upload = this.uploads.get(uploadId);
    return upload && upload.userId === userId ? upload : null;
  }

  async markFilesUploaded(userId: string, uploadId: string): Promise<void> {
    const upload = await this.getUserUpload(userId, uploadId);
    if (upload) {
      upload.filesUploadedAt = now();
      upload.updatedAt = now();
    }
  }
}

function createUploadStore(): UploadStore {
  if (config.authStore === "postgres") {
    if (!config.databaseUrl) {
      throw new Error("DATABASE_URL is required when AUTH_STORE=postgres.");
    }

    return new PostgresUploadStore(config.databaseUrl);
  }

  return new MemoryUploadStore();
}

export const uploadStore = createUploadStore();
export const createUploadId = () => randomUUID();
export const createUploadFileId = () => randomUUID();
