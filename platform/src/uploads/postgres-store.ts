import { Pool } from "pg";

import { poolConfig } from "../db/pool.js";
import type {
  CreateUploadInput,
  UploadFileRecord,
  UploadRecord,
  UploadStatus,
  UploadStore
} from "./store.js";

type UploadRow = {
  id: string;
  user_id: string;
  prompt: string;
  source_url: string | null;
  status: UploadStatus;
  upload_prefix: string;
  result_object_name: string | null;
  result_file_name: string | null;
  result_content_type: string | null;
  files_uploaded_at: Date | null;
  completed_at: Date | null;
  created_at: Date;
  updated_at: Date;
};

type UploadFileRow = {
  id: string;
  upload_id: string;
  original_name: string;
  object_name: string;
  content_type: string;
  size_bytes: string;
  created_at: Date;
};

function mapFile(row: UploadFileRow): UploadFileRecord {
  return {
    id: row.id,
    uploadId: row.upload_id,
    originalName: row.original_name,
    objectName: row.object_name,
    contentType: row.content_type,
    sizeBytes: Number(row.size_bytes),
    createdAt: row.created_at
  };
}

function mapUpload(row: UploadRow, files: UploadFileRecord[]): UploadRecord {
  return {
    id: row.id,
    userId: row.user_id,
    prompt: row.prompt,
    sourceUrl: row.source_url ?? undefined,
    status: row.status,
    uploadPrefix: row.upload_prefix,
    resultObjectName: row.result_object_name ?? undefined,
    resultFileName: row.result_file_name ?? undefined,
    resultContentType: row.result_content_type ?? undefined,
    filesUploadedAt: row.files_uploaded_at ?? undefined,
    completedAt: row.completed_at ?? undefined,
    createdAt: row.created_at,
    updatedAt: row.updated_at,
    files
  };
}

export class PostgresUploadStore implements UploadStore {
  private readonly pool: Pool;

  constructor(databaseUrl: string) {
    this.pool = new Pool(poolConfig(databaseUrl));
  }

  async createUpload(input: CreateUploadInput): Promise<UploadRecord> {
    const client = await this.pool.connect();

    try {
      await client.query("begin");

      const uploadResult = await client.query<UploadRow>(
        `insert into shotwell_uploads (id, user_id, prompt, source_url, upload_prefix)
        values ($1, $2, $3, $4, $5)
        returning
          id,
          user_id,
          prompt,
          source_url,
          status,
          upload_prefix,
          result_object_name,
          result_file_name,
          result_content_type,
          files_uploaded_at,
          completed_at,
          created_at,
          updated_at`,
        [input.id, input.userId, input.prompt, input.sourceUrl ?? null, input.uploadPrefix]
      );

      const fileRows: UploadFileRecord[] = [];
      for (const file of input.files) {
        const fileResult = await client.query<UploadFileRow>(
          `insert into shotwell_upload_files (
            id,
            upload_id,
            original_name,
            object_name,
            content_type,
            size_bytes
          ) values ($1, $2, $3, $4, $5, $6)
          returning id, upload_id, original_name, object_name, content_type, size_bytes, created_at`,
          [file.id, input.id, file.originalName, file.objectName, file.contentType, file.sizeBytes]
        );
        fileRows.push(mapFile(fileResult.rows[0]));
      }

      await client.query("commit");
      return mapUpload(uploadResult.rows[0], fileRows);
    } catch (error) {
      await client.query("rollback");
      throw error;
    } finally {
      client.release();
    }
  }

  async listUserUploads(userId: string): Promise<UploadRecord[]> {
    const uploadResult = await this.pool.query<UploadRow>(
      `select
        id,
        user_id,
        prompt,
        source_url,
        status,
        upload_prefix,
        result_object_name,
        result_file_name,
        result_content_type,
        files_uploaded_at,
        completed_at,
        created_at,
        updated_at
      from shotwell_uploads
      where user_id = $1
      order by created_at desc
      limit 50`,
      [userId]
    );

    return this.hydrateUploads(uploadResult.rows);
  }

  async getUserUpload(userId: string, uploadId: string): Promise<UploadRecord | null> {
    const uploadResult = await this.pool.query<UploadRow>(
      `select
        id,
        user_id,
        prompt,
        source_url,
        status,
        upload_prefix,
        result_object_name,
        result_file_name,
        result_content_type,
        files_uploaded_at,
        completed_at,
        created_at,
        updated_at
      from shotwell_uploads
      where id = $1 and user_id = $2`,
      [uploadId, userId]
    );

    const upload = (await this.hydrateUploads(uploadResult.rows))[0];
    return upload ?? null;
  }

  async markFilesUploaded(userId: string, uploadId: string): Promise<void> {
    await this.pool.query(
      `update shotwell_uploads
      set files_uploaded_at = now(), updated_at = now()
      where id = $1 and user_id = $2`,
      [uploadId, userId]
    );
  }

  async updatePendingPrompt(userId: string, uploadId: string, prompt: string): Promise<boolean> {
    const result = await this.pool.query(
      `update shotwell_uploads
      set prompt = $3, updated_at = now()
      where id = $1 and user_id = $2 and status = 'pending'`,
      [uploadId, userId, prompt]
    );

    return result.rowCount === 1;
  }

  private async hydrateUploads(rows: UploadRow[]): Promise<UploadRecord[]> {
    if (rows.length === 0) {
      return [];
    }

    const ids = rows.map((row) => row.id);
    const fileResult = await this.pool.query<UploadFileRow>(
      `select id, upload_id, original_name, object_name, content_type, size_bytes, created_at
      from shotwell_upload_files
      where upload_id = any($1::uuid[])
      order by created_at asc, original_name asc`,
      [ids]
    );

    const filesByUploadId = new Map<string, UploadFileRecord[]>();
    for (const fileRow of fileResult.rows) {
      const files = filesByUploadId.get(fileRow.upload_id) ?? [];
      files.push(mapFile(fileRow));
      filesByUploadId.set(fileRow.upload_id, files);
    }

    return rows.map((row) => mapUpload(row, filesByUploadId.get(row.id) ?? []));
  }
}
